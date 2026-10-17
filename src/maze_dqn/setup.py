from setuptools import find_packages, setup

setup(
    name='maze_dqn', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/maze_dqn']),
                ('share/maze_dqn', ['package.xml', 'README.md', 'LICENSE'])],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='Maze project', maintainer_email='maintainer@example.com',
    description='Reward-trained DQN global routes with Nav2 local path execution',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'train_dqn = maze_dqn.train:main',
        'evaluate_dqn = maze_dqn.evaluate:main',
    ]},
)
