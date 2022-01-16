import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rrtstar",
    version="0.0.1",
    author="Romain Desarzens",
    author_email="rdesarz.pro@pm.me",
    description="A RRT star implementation in Python",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/rdesarz/rrtstar",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    entry_points={"console_scripts": ["rrtstar=rrtstar.command_line:main"]},
    python_requires='>=3.6',
)
