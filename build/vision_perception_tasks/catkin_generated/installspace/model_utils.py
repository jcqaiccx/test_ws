#!/usr/bin/env python3
import os
import urllib.request

MODEL_FILES = {
    "deploy.prototxt": "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt",
    "mobilenet_iter_73000.caffemodel": "https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel",
}


def ensure_model_files(model_dir):
    os.makedirs(model_dir, exist_ok=True)
    local_paths = {}

    for filename, url in MODEL_FILES.items():
        dst = os.path.join(model_dir, filename)
        if not os.path.exists(dst):
            urllib.request.urlretrieve(url, dst)
        local_paths[filename] = dst

    return local_paths
