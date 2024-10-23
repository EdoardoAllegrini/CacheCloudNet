import json
import os


path = r"/home/intou/Desktop/IoT/IoT-Project-2024/src/output_tests/"
counter = 0
for file in os.listdir(path):
    if file.endswith(".json"):
        with open(path + file) as f:
            content = json.load(f)
        if not content["queries"]:
            os.remove(path + file)
            counter += 1

print(f"{counter} files were cleaned")
