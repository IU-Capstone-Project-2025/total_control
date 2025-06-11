from setuptools import setup, find_packages

setup(
    name="inno_control",
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["pyserial>=3.5"],
)
