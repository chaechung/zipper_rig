import json

file_path = r"D:\Dropbox\PythonScripts\012_ZipRig\jason_data\ctrl_shape_data.json"

with open(file_path, "r") as file_for_read:
    data = json.load(file_for_read)

print(data)