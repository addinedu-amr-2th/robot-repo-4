import os
from setuptools import setup

package_name = "audio_subscriber"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="melebele3073",
    maintainer_email="melebele3073@gmail.com",
    description="subscribe and play audio data",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_subscriber = audio_subscriber.audio_subscriber:main",
            "stt_subscriber = audio_subscriber.stt_subscriber:main",
        ],
    },
)