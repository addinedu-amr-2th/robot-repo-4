from setuptools import setup
import os
from glob import glob

package_name = 'stt_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yun',
    maintainer_email='yun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_pub=stt_pkg.audio_pub:main',
            'audio_sub=stt_pkg.audio_sub:main',
            'stt_sub=stt_pkg.stt_sub:main',
            'gpt_agent=stt_pkg.gpt_agent:main',
            'tts_pub=stt_pkg.tts_pub:main',
        ],
    },
)
