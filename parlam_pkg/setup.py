from setuptools import find_packages, setup
import glob 
import os

package_name = 'parlam_pkg'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [file_path]))
    return paths

data_files = [
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
]

# Add all files in data/ recursively
data_files += package_files('data')
# Add all files in models/ recursively
data_files += package_files('models')
# Add all files in conversations/ recursively
data_files += package_files('conversations')


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhriscu',
    maintainer_email='lavinia.beatrice.hriscu@upc.edu',
    description='PARLAM framework nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "conversation_server= parlam_pkg.conversation_server: main",
            "speech_input_server= parlam_pkg.speech_input_server: main",
            "llm_server= parlam_pkg.llm_server: main",
            "speech_output_server= parlam_pkg.speech_output_server: main"
        ],
    },
)
