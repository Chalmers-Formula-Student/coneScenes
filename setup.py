from setuptools import setup

setup(
    name="conScenes",
    version="0.1",
    packages=["cli"],
    package_data={'': ['data.json']},
    include_package_data=True,
    install_requires=[
        "click",
        "termcolor",
        "datetime",
    ],
    entry_points="""
        [console_scripts]
        conescenes=cli.conescenes:conescenes
    """,
)
