from setuptools import find_packages, setup

package_name = 'orion_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoanganh',
    maintainer_email='hoanganh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ai_bridge_node = orion_ai.ai_bridge_node:main',
            'stt_node = orion_ai.stt_node:main',
            'tts_node = orion_ai.tts_node:main',
        ],
    },
)
