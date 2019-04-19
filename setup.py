import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyemir",
    version="0.0.1",
    author="Zoran Benic",
    author_email="",
    description="Class Emir used for controlling the FMENA eMIR robot.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/zbenic/emir",
    install_requires=[
        'PyBluez-win10',
    ],
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
