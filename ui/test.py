import re

line = "frame_id[22][33]" # set frame & scene

elements = re.findall('frame_id\[(.*?)\]\[(.*?)\]', "frame_id[22][33]")[0]
frame_id, scene_id = map(int, elements)

line = "|1|2|3|"
for d in line.strip().split("|"):
    if not d:continue
    print(d)