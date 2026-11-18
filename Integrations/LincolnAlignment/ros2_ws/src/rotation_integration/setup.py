from setuptools import setup
from glob import glob

package_name = 'rotation_integration'
setup(name=package_name, version='0.3.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Project maintainers', maintainer_email='maintainers@example.com',
      description='One-shot Lincoln Autoware to Unity map alignment', license='Apache-2.0',
      entry_points={'console_scripts': [
          'snapshot = rotation_integration.snapshot_node:main',
          'file_preview = rotation_integration.file_preview:main',
          'initialize_once = rotation_integration.initialize_once:main',
          'align_maps = rotation_integration.map_alignment_node:main']})
