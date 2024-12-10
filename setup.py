from setuptools import setup, find_packages
import os

# Read the requirements from requirements.txt
def read_requirements():
    with open("requirements.txt") as req:
        return req.read().splitlines()

setup(
    name="Harvesting",  
    version="0.1.0",
    author="Sai Navaneet",
    author_email="sainavaneet@knu.ac.kr",
    description="A package for the harvesting robot project",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/sainavaneet/Harvesting",  
    packages=find_packages(include=["base_control", "object_detection"]),
    package_data={
        '': ['config/*.yaml', 'rviz/*.rviz', 'launch/*.launch']
    },
    include_package_data=True,
    install_requires=read_requirements(),  # Automatically load dependencies from requirements.txt

    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8, <3.9",
)
