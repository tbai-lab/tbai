import os
import sys
import subprocess
from pathlib import Path
import importlib

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

        # 1. Re-use torch's libraries if available
        # 2. torch._C._GLIBCXX_USE_CXX11_ABI evaluates to False for older versions
        # 3. we need to use the same ABI as torch to avoid symbol conflicts
        # 4. the block bellow will only succeed if --build-no-isolation is used (otherwise torch is not available)
        try:
            torch = importlib.import_module("torch")
            torch_abi = int(torch.compiled_with_cxx11_abi())
            if torch_abi != 1:
                raise ValueError(f"[tbai] torch ABI {torch_abi} is not supported, current version is {torch.__version__}. Upgrade to torch>=2.7.0")
            torch_cmake_dir = Path(torch.utils.cmake_prefix_path) / "Torch"
            cmake_args.append(f"-DTorch_DIR={torch_cmake_dir}")
            print(f"[tbai] torch found at {torch_cmake_dir} with -D_GLIBCXX_USE_CXX11_ABI {torch_abi}")

        except ImportError:
            print("[tbai] torch not importable; relying on CMAKE_PREFIX_PATH")

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
