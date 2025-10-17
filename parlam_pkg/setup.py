from setuptools import find_packages, setup
import glob 

package_name = 'parlam_pkg'

files=glob.glob('models/**/*',recursive=True)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/handover.txt']),
        ('share/' + package_name + '/data', ['data/info.txt']),
        ('share/' + package_name + '/data', ['data/info_eng.pdf']),
        ('share/' + package_name + '/data', ['data/greetings.json']),
        ('share/' + package_name + '/models/vosk-model-small-ca-0.4/am',glob.glob('models/vosk-model-small-ca-0.4/am/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-small-ca-0.4/conf',glob.glob('models/vosk-model-small-ca-0.4/conf/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-small-ca-0.4/graph',glob.glob('models/vosk-model-small-ca-0.4/graph/*.*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-small-ca-0.4/graph/phones',glob.glob('models/vosk-model-small-ca-0.4/graph/phones/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-small-ca-0.4/ivector',glob.glob('models/vosk-model-small-ca-0.4/ivector/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/am',glob.glob('models/vosk-model-en-us-0.22/am/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/conf',glob.glob('models/vosk-model-en-us-0.22/conf/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/graph',glob.glob('models/vosk-model-en-us-0.22/graph/*.*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/graph/phones',glob.glob('models/vosk-model-en-us-0.22/graph/phones/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/ivector',glob.glob('models/vosk-model-en-us-0.22/ivector/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/rescore',glob.glob('models/vosk-model-en-us-0.22/rescore/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-en-us-0.22/rnnlm',glob.glob('models/vosk-model-en-us-0.22/rnnlm/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/am',glob.glob('models/vosk-model-es-0.42/am/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/conf',glob.glob('models/vosk-model-es-0.42/conf/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/graph',glob.glob('models/vosk-model-es-0.42/graph/*.*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/graph/phones',glob.glob('models/vosk-model-es-0.42/graph/phones/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/ivector',glob.glob('models/vosk-model-es-0.42/ivector/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/rescore',glob.glob('models/vosk-model-es-0.42/rescore/*',recursive=True)),
        ('share/' + package_name + '/models/vosk-model-es-0.42/rnnlm',glob.glob('models/vosk-model-es-0.42/rnnlm/*',recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhriscu',
    maintainer_email='lavinia.beatrice.hriscu@upc.edu',
    description='PARLAM framework nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "parlam_client= parlam_pkg.parlam_client: main",
            "speech_input_server= parlam_pkg.speech_input_server: main",
            "llm_server= parlam_pkg.llm_server: main",
            "speech_output_server= parlam_pkg.speech_output_server: main"
        ],
    },
)
