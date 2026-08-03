from glob import glob

from setuptools import find_packages, setup


package_name = "voice_llm_agent"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="luong",
    maintainer_email="luuluong2000hh@gmail.com",
    description=(
        "Domain and application engine for a tool-calling voice robot "
        "assistant."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tool_executor = "
            "voice_llm_agent.infrastructure.ros.tool_executor_node:main",
        ],
    },
)
