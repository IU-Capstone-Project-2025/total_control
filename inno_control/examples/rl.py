import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# Подразумевается, что библиотека inno_control установлена и доступна
# pip install inno-control-devices
from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

# Установка Stable-Baselines3:
# pip install stable-baselines3[extra]
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env

# ---------------------- Улучшенная Gym-среда для реального устройства ----------------------

class RealCartPoleEnv(gym.Env):
    """
    Gymnasium-среда для управления реальным физическим маятником на тележке.
    Ключевые улучшения:
    - Плотная функция награды для эффективного обучения.
    - Корректная обработка ошибок связи с устройством.
    - Условия завершения эпизода по положению тележки и углу маятника.
    """
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, port=None):
        super().__init__()
        
        # --- Подключение к устройству ---
        try:
            if port:
                device_port = port
            else:
                print("Сканирование портов для поиска устройства...")
                device_port = find_your_device()
            
            if device_port is None:
                raise ConnectionError("Устройство не найдено. Проверьте подключение.")

            print(f"Подключение к устройству на порту: {device_port}")
            self.device = CartPole(device_port)
            self.device.connect(do_init_activity=False)
        except Exception as e:
            print(f"Ошибка при инициализации устройства: {e}")
            raise

        # --- Пространство действий ---
        # Непрерывное действие: усилие (ток), которое подается на мотор.
        # Диапазон от -120 до 120, как в вашем примере.
        self.action_space = spaces.Box(low=-120.0, high=120.0, shape=(1,), dtype=np.float32)

        # --- Пространство состояний (наблюдений) ---
        # Состояние: [theta, theta_dot, p, p_dot]
        # theta: угол маятника (рад), theta_dot: угловая скорость (рад/с)
        # p: положение тележки (м), p_dot: скорость тележки (м/с)
        high = np.array([
            np.pi,              # Макс. угол
            np.finfo(np.float32).max, # Макс. угловая скорость
            0.4,                # Макс. положение (подберите под вашу установку)
            np.finfo(np.float32).max  # Макс. скорость
        ], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        # --- Параметры для награды и завершения ---
        self.angle_threshold = 30.0 * (np.pi / 180.0) # 30 градусов
        self.position_threshold = 0.35 # 35 см от центра (подберите под вашу установку)

    def _get_obs(self):
        """Читает и парсит состояние с устройства."""
        s = self.device.get_joint_state()
        if s is None:
            # ВАЖНО: Если данные не пришли, это ошибка. Прерываемся.
            raise ConnectionError("Не удалось получить состояние от устройства.")
        
        # Парсинг строки и конвертация в нужные единицы
        vals = list(map(float, s.split()))
        theta = vals[0] * (np.pi / 180.0) # градусы -> радианы
        theta_dot = vals[1] * (np.pi / 180.0) # град/с -> рад/с
        p = vals[2] / 1000.0 # мм -> м
        p_dot = vals[3] / 1000.0 # мм/с -> м/с
        return np.array([theta, theta_dot, p, p_dot], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Сброс устройства в начальное положение
        print("Сброс положения устройства...")
        self.device.re_init()
        # Даем устройству время "успокоиться" после сброса
        time.sleep(1.0)
        
        try:
            obs = self._get_obs()
        except ConnectionError:
            # Если даже при сбросе не можем получить состояние, делаем вид, что все плохо
            obs = np.array([np.pi, 0, 0, 0], dtype=np.float32)

        return obs, {}

    def step(self, action):
        try:
            # 1. Выполняем действие
            u = float(np.clip(action, self.action_space.low, self.action_space.high))
            self.device.set_joint_efforts(u)

            # Частота управления ~50 Гц.
            time.sleep(0.02) 

            # 2. Получаем новое состояние
            obs = self._get_obs()
            theta, _, p, _ = obs

            # 3. Определяем, не закончился ли эпизод
            done = bool(
                abs(theta) > self.angle_threshold
                or abs(p) > self.position_threshold
            )

            # 4. Рассчитываем "плотную" награду
            if not done:
                # Награда за удержание угла (cos(0)=1, cos(pi/6)=0.86)
                reward_angle = np.cos(theta)
                # Награда за удержание в центре (экспонента от 1 до 0)
                reward_position = np.exp(-(p / self.position_threshold)**2)
                # Комбинированная награда
                reward = reward_angle + 0.5 * reward_position
            else:
                # Большой штраф за падение или выход за пределы
                reward = -10.0
            
            # Gymnasium требует 5 возвращаемых значений
            return obs, reward, done, False, {}

        except ConnectionError as e:
            print(f"Ошибка связи в step: {e}. Завершение эпизода.")
            # Если связь потеряна, аварийно завершаем эпизод
            dummy_obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            reward = -20.0 # Еще больший штраф за потерю связи
            return dummy_obs, reward, True, False, {}

    def render(self):
        # Визуализация происходит на самом устройстве, здесь ничего не делаем.
        pass

    def close(self):
        print("Остановка эксперимента и отключение от устройства.")
        self.device.stop_experiment()
        self.device.disconnect()


# ---------------------- Основной скрипт для обучения агента ----------------------
if __name__ == '__main__':
    # --- 1. Создание и проверка среды ---
    # !!! ВАЖНО: Укажите здесь порт вашего устройства или оставьте None для автопоиска !!!
    # Пример для Linux: '/dev/ttyUSB0', для Windows: 'COM3'
    DEVICE_PORT = None 
    
    env = RealCartPoleEnv(port=DEVICE_PORT)
    # Проверка, что среда соответствует стандарту Gymnasium
    check_env(env)
    
    # --- 2. Создание RL-агента ---
    # Мы используем Soft Actor-Critic (SAC) - мощный алгоритм для непрерывных действий.
    model = SAC(
        "MlpPolicy",          # Используем стандартную нейросеть (Multi-Layer Perceptron)
        env,
        verbose=1,            # Выводить информацию о процессе обучения
        learning_starts=1000, # Начать обучение после 1000 шагов со случайными действиями
        buffer_size=50000,    # Хранить 50000 последних переходов в памяти
        gamma=0.98,           # Коэффициент дисконтирования (насколько важны будущие награды)
        tau=0.01,             # Коэффициент "мягкого" обновления сетей
        train_freq=(1, "step")# Обучаться после каждого шага
    )

    # --- 3. Обучение ---
    print("\n" + "="*50)
    print("!!! ВНИМАНИЕ: НАЧИНАЕТСЯ ОБУЧЕНИЕ НА РЕАЛЬНОМ УСТРОЙСТВЕ !!!")
    print("Будьте готовы остановить скрипт (Ctrl+C), если что-то пойдет не так.")
    print("="*50 + "\n")
    time.sleep(3)
    
    try:
        # Укажите желаемое количество шагов для обучения. Начните с 20000.
        TOTAL_TIMESTEPS = 50000
        model.learn(total_timesteps=TOTAL_TIMESTEPS, log_interval=10)
        
        # --- 4. Сохранение модели ---
        model.save("sac_real_cartpole_model")
        print(f"\nОбучение завершено! Модель сохранена в файл 'sac_real_cartpole_model.zip'")
        
    except KeyboardInterrupt:
        print("\nОбучение прервано пользователем.")
    except Exception as e:
        print(f"\nВо время обучения произошла ошибка: {e}")
    finally:
        # --- 5. Корректное завершение ---
        # Этот блок выполнится всегда: при успехе, ошибке или прерывании.
        env.close()
        
