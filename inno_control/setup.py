from setuptools import setup, find_packages

setup(
    name="InnoControl",
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.22",
        "scipy>=1.10",
        "matplotlib>=3.7",
        "control>=0.9",
        "gymnasium>=0.28",
        "mujoco>=3.3",
        "torch>=2.0",
        "pyserial>=3.5"
    ],
    author="Dungeon team",
    author_email="e.shlomov@innopolis.university",
    description="Lib for Innopolis Robotics Lab control platforms",
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/IU-Capstone-Project-2025/total_control",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)