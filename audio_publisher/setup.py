from setuptools import setup

package_name = 'audio_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={
        'audio_publisher': [
            'audio_publisher/mic_array.py',
            'audio_publisher/file_parser.py',
            'audio_publisher/constants.py',
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duembgen',
    maintainer_email='frederike.duembgen@epfl.ch',
    description='Publishing of audio data from different sources',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream = audio_publisher.stream_publisher:main'
        ],
    },
)
