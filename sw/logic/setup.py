"""Build script for Cython extensions inside logic/."""

from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

extensions = [
    Extension(
        "localization._bear_detector_fast",
        sources=["localization/_bear_detector_fast.pyx"],
        include_dirs=[numpy.get_include()],
        extra_compile_args=["-O3"],
    ),
]

setup(
    name="logic-cython",
    ext_modules=cythonize(
        extensions,
        compiler_directives={"language_level": "3"},
    ),
)
