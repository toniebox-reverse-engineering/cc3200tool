from distutils.core import setup

setup(
    name="cc3200tool",
    version="1.2.4",
    description="A tool to down-/upload files form/to TI CC3200",
    author="Kiril Zyapkov, 0xbadbee",
    author_email="k.zyapkov@allterco.com",
    url="https://github.com/toniebox-reverse-engineering/cc3200tool",
    packages=['cc3200tool'],
    package_data={'cc3200tool': ['dll/*.dll']},
    entry_points = {
        'console_scripts': ['cc3200tool=cc3200tool.cc:main'],
    },
    install_requires=['pyserial'],
)
