import os
import shutil
import sys
import subprocess
from pathlib import Path

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        cfg = "Release"

        build_go2_unitree = str(os.environ.get('TBAI_BUILD_DEPLOY_GO2_UNITREE', 'OFF')).upper()
        build_g1_unitree = str(os.environ.get('TBAI_BUILD_DEPLOY_G1_UNITREE', 'OFF')).upper()

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DBUILD_SHARED_LIBS=ON",
            "-DTBAI_BUILD_PYTHON=ON",
            f"-DTBAI_BUILD_DEPLOY_GO2_UNITREE={build_go2_unitree}",
            f"-DTBAI_BUILD_DEPLOY_G1_UNITREE={build_g1_unitree}",
        ]

        build_args = []

        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]
            else:
                build_args += [f"-j{os.cpu_count()}"]

        build_temp = Path(self.build_temp) / ext.name
        build_temp.mkdir(parents=True, exist_ok=True)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args],
            cwd=build_temp,
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", ".", *build_args],
            cwd=build_temp,
            check=True,
        )

        # CMAKE_LIBRARY_OUTPUT_DIRECTORY covers normal targets but misses IMPORTED libs
        # (onnxruntime) and ExternalProject artifacts (qpOASES). Stage an install and
        # mirror its lib/ .so* files next to _C.so so the loader finds everything.
        staging = (build_temp / "_install_staging").resolve()
        if staging.exists():
            shutil.rmtree(staging)
        result = subprocess.run(
            ["cmake", "--install", ".", "--prefix", str(staging)],
            cwd=build_temp,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            sys.stdout.write(result.stdout)
            sys.stderr.write(result.stderr)
            result.check_returncode()
        for src in (staging / "lib").glob("*.so*"):
            dst = extdir / src.name
            if dst.exists() or dst.is_symlink():
                dst.unlink()
            shutil.copy2(src, dst, follow_symlinks=False)

    def run(self):
        super().run()

        build_lib = Path(self.build_lib).resolve()
        env = os.environ.copy()
        env["PYTHONPATH"] = str(build_lib)

        result = subprocess.run(
            [sys.executable, "-m", "nanobind.stubgen", "-m", "tbai._C", "-r", "-q"],
            env=env,
            cwd=str(build_lib),
        )
        if result.returncode != 0:
            print("WARNING: stub generation failed — skipping. IDE type hints for tbai._C will not be available.")


setup(
    name="tbai",
    version="2.0.0",
    author="TBAI Lab",
    description="Towards Better Athletic Intelligence",
    packages=find_packages(),
    ext_modules=[CMakeExtension("tbai._C", sourcedir="..")],
    cmdclass={"build_ext": CMakeBuild},
    python_requires=">=3.10",
    install_requires=["numpy"],
    setup_requires=["nanobind"],
    zip_safe=False,
)
