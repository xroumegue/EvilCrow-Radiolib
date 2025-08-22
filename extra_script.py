Import("env")
from pathlib import Path
from subprocess import run
from shutil import copyfile

build_dir=Path(env["PROJECT_BUILD_DIR"]).joinpath(env["PIOENV"])
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

linkFlags = str(env.Dictionary("LINKFLAGS")).replace("-T sections.ld", "-T sections-ecrf.ld")
env.Replace(LINKFLAGS=linkFlags)
sectionFile = None
sectionLibPath =None
for libpath in env.Dictionary("LIBPATH"):
    sectionFile=Path(libpath).joinpath("sections.ld")
    if (sectionFile.exists()):
        print(sectionFile)
        sectionLibPath = libpath 
        break

cp = run([
    'patch',
    '-i',
    Path(env["PROJECT_DIR"]).joinpath("etc").joinpath("ld").joinpath("sections.ld.patch"),
    '-o',
    build_dir.joinpath("sections-ecrf.ld")
    ],
    cwd=sectionLibPath
)
