import setuptools

# read the contents of your README file
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setuptools.setup(
    name='abb_robot_client',
    version='0.2.0',
    description='Python client library to access ABB robots using RWS and EGM',
    url='https://github.com/johnwason/abb_robot_client',
    packages=setuptools.find_packages("src"),
    package_dir={"" :"src"},
    install_requires=[
        'setuptools',
        'requests',
        'numpy',
        'protobuf',
        'websocket-client'
    ],
    extras_require={
        "aio": ["httpx", "websockets"],
        "testing": ["pytest", "pytest-asyncio"],
    },
    long_description=long_description,
    long_description_content_type='text/markdown'
)