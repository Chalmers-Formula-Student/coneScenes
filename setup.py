from setuptools import setup

setup(
    name="conescenes",
    version="1.0.0",
    packages=["cli"],
    package_data={'': ['data.json']},
    include_package_data=True,
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    install_requires=[
        "click",
        "termcolor",
        "datetime",
        "zipfile",
        "tkinter",
        "numpy",
        "json",
    ],
    entry_points="""
        [console_scripts]
        conescenes=cli.conescenes:conescenes
    """,
)
