from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ollama_image_understanding'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ 安装 launch 文件
        (os.path.join('share', package_name,
                      'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ming-wsl',
    maintainer_email='ming-zhanglu@outlook.com',
    description='图像理解功能包，使用 Ollama 模型进行图像内容描述',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ollama_understanding = ollama_image_understanding.ollama_understanding:main',
        ],
    },
)
