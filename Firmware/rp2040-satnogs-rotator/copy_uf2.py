Import("env")
import shutil
import os

def copy_uf2(*args, **kwargs):
    src = os.path.join(env.subst("$BUILD_DIR"), "firmware.uf2")
    dst = os.path.join(env.subst("$PROJECT_DIR"), "..", "..", "firmware.uf2")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"UF2 copied to {os.path.abspath(dst)}")

env.AddPostAction("buildprog", copy_uf2)
