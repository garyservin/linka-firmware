# PlatformIO pre-build script: sets the firmware VERSION from git.
#
# - LINKA_VERSION env var, if set (CI release builds), e.g. "0.3.3"
# - otherwise `git describe` of the latest version tag, e.g. "0.3.2" on a tagged
#   commit or "0.3.2-4-gabc1234-dirty" for development builds
# - "0.0.0-dev" when git is not available
import os
import subprocess

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)


def get_version():
    version = os.environ.get("LINKA_VERSION")
    if version:
        return version.removeprefix("v")
    try:
        described = subprocess.check_output(
            ["git", "describe", "--tags", "--dirty",
             "--match", "v[0-9]*", "--match", "[0-9]*"],
            cwd=env["PROJECT_DIR"],  # noqa: F821
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "0.0.0-dev"
    return described.removeprefix("v")


version = get_version()
print(f"Linka firmware version: {version}")
env.Append(CPPDEFINES=[("VERSION", env.StringifyMacro(version))])  # noqa: F821
