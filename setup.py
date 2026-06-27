"""Legacy setuptools entrypoint.

Prefer building/installing via pyproject.toml.
"""

from setuptools import setup, find_packages

setup(
    name="KarlsPlayground",
    version="1.0.0",
    author="Karl",
    description="A realistic Audi A5 parking simulation environment",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.20.0",
        "matplotlib>=3.4.0",
    ],
    keywords="parking simulation vehicle control audi a5 autonomous driving",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "License :: OSI Approved :: MIT License",
    ],
)
