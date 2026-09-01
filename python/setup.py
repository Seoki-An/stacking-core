from pathlib import Path
import os
import subprocess
import sys

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


ROOT = Path(__file__).resolve().parents[1]


class CMakeExtension(Extension):
    def __init__(self, name: str) -> None:
        super().__init__(name, sources=[])


class CMakeBuild(build_ext):
    def build_extension(self, extension: Extension) -> None:
        output = Path(self.get_ext_fullpath(extension.name)).parent.resolve()
        temporary = Path(self.build_temp) / extension.name
        temporary.mkdir(parents=True, exist_ok=True)

        build_type = "Debug" if self.debug else "Release"
        configure = [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(temporary),
            f"-DCMAKE_BUILD_TYPE={build_type}",
            "-DBUILD_TESTING=OFF",
            "-DSTACKING_CORE_BUILD_PYTHON=ON",
            f"-DSTACKING_CORE_PYTHON_OUTPUT_DIR={output}",
            f"-DPython3_EXECUTABLE={sys.executable}",
        ]
        subprocess.run(configure, check=True)

        build = [
            "cmake",
            "--build",
            str(temporary),
            "--target",
            "stacking-core-python",
        ]
        parallel = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        if not parallel and self.parallel:
            build.extend(["-j", str(self.parallel)])
        subprocess.run(build, check=True)


setup(
    ext_modules=[CMakeExtension("stacking_core._native")],
    cmdclass={"build_ext": CMakeBuild},
    packages=["stacking_core"],
)
