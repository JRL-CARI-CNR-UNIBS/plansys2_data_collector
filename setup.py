import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'plansys2_data_collector'

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['config/', 'launch/', 'results/']),
    install_requires=['setuptools', 'bag_to_csv'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_generator = plansys2_data_collector.plot_generator:main',
            'plan_plot_generator = plansys2_data_collector.plan_plot_generator:main',
            'plan_plot_comparison_generator = plansys2_data_collector.plan_plot_comparison_generator:main',
        ],
    },
)
