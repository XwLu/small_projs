
import os
import re
import time
def load_all(file_path):
    data = {}
    with open(file_path, 'r') as file: #打开文件
        lines = file.readlines()
        for line in lines:
            elements = re.findall('frame_id\[(.*?)\]\[(.*?)\]', line[:20])[:1][0]
            frame_num, scene_idx = map(int, elements)
            if frame_num not in data:
                data[frame_num] = {}
            if scene_idx not in data[frame_num]:
                data[frame_num][scene_idx] = {}
            data[frame_num][scene_idx] = decode(line)
    return data

def decode(line):
    result = {}
    id_list = line.strip().split("$")
    for id_data in id_list:
        elements = re.findall('<(.*?)>', id_data)
        if elements == []:
            continue
        id = int(elements[0])
        data = id_data.strip().split("|")

        def get_list_data(data):
            list_data = []
            for d in data.strip().rstrip('*').split("/"):
                if not d: continue
                list_data.append(tuple(map(float, d.strip().split(","))))
            return list_data
    
        polygon = get_list_data(data[1])
        refpath = get_list_data(data[2])
        traj = get_list_data(data[3])
        cost = {}
        if data[4]:
            cost_data = data[4].split(",")
            for cd in cost_data[:-1]:
                name, value = cd.split(":")
                cost[name] = float(value)
            cost["simu_result"] = {}
            for s in [d for d in cost_data[-1].strip().rstrip('*').split("!") if d]:
                s_cost = ''
                decision = ''
                source = ''
                sub_decision = ''
                sub_agent_data = s.split('^')
                sub_agent_data = [d for d in sub_agent_data if d != ""]
                if len(sub_agent_data) == 2:
                    subid, s_cost = sub_agent_data
                elif len(sub_agent_data) == 3:
                    subid, decision, source = sub_agent_data
                elif len(sub_agent_data) == 4:
                    subid, s_cost, decision, source = sub_agent_data
                elif len(sub_agent_data) == 5:
                    subid, s_cost, decision, source, sub_decision = sub_agent_data
                else:
                    assert False, f"WRONG SIMU DATA LEN, {len(sub_agent_data)}, {[d for d in sub_agent_data]}"
                cost["simu_result"][int(subid)] = (s_cost, decision, source, sub_decision)
    
        other = {}
        if data[5]:
            scene_reward, softmax_prob, label, roi_l = [d for d in data[5].strip().rstrip('*').split("/") if d]
            other["scene_reward"] = (scene_reward)
            other["softmax_prob"] = (softmax_prob)
            other["refpath_label"] = label
            other["refpath_roi_l"] = (roi_l)
    
        result[id] = {'polygon': polygon, 'refpath': refpath, 'traj': traj, 'cost': cost, 'other': other}
    return result

size = 4
r = "<font color=red size="+str(size)+"><b>{}</font>"
b = "<font color=blue size="+str(size)+"><b>{}</font>"
y = "<font color=orange size="+str(size)+"><b>{}</font>"
g = "<font color=green size="+str(size)+"><b>{}</font>"
z = "<font color=black size="+str(size)+"><b>{}</font>"
# style=\"background-color: yellow; white-space:pre;
def encode_html(data):
    ego_data = data[0]['other']
    text = """
    <font color=black size={size}><b>
    Scene reward[{scene_reward}]{blank}Prob[{softmax_prob}]{blank}Label[{refpath_label}]{blank}Roi[{refpath_roi_l}]
    </font>
    """.format(
        size = size,
        scene_reward = ego_data['scene_reward'],
        softmax_prob = ego_data['softmax_prob'],
        refpath_label = ego_data['refpath_label'],
        refpath_roi_l = ego_data['refpath_roi_l'],
        blank="&nbsp;&nbsp;&nbsp;"
    )
    i = 0
    for id, id_data in data.items():
        cost = id_data['cost']
        if cost == {}: continue
        def get_cost (name): return cost[name] if name in cost else "DNone"
        if 'total_cost' not in cost: continue
        if i % 2 == 0:
            text += "<p style=\"background-color: rgb(225, 255, 255);\">"
        else:
            text += "<p>"
        i += 1
        text += """
        <font color=black size={size}><b>
        {id}{blank}total[{total_cost}]{blank}efficiency|{efficiency_cost}]{blank}comfort[{comfort_cost}]{blank}\
        navi[{navigation_cost}]{blank}safety[{safety_cost}]{blank}post[{postprocess_cost}]{blank}
        <br>
        </font>
        <font color=green size={size}><b>
        v_end[{v_end}]{blank}v_front[{v_front}]{blank}s_end[{s_end}]{blank}block_s[{block_s}]{blank}
        <br>
        </font>
        <font color=orange size={size}><b>
        max_l[{max_l}]{blank}max_offset[{max_offset}]{blank}max_deltheta[{max_deltheta}]{blank}end_l[{end_l}]{blank}\
        max_expand[{max_expand}]{blank}
        <br>
        lat_acc[{lat_acc}]{blank}lon_acc[{lon_acc}]{blank}lon_jerk[{lon_jerk}]{blank}refpath_dist[{refpath_dist}]{blank}\
        max_kappa[{max_kappa}]{blank}
        <br>
        refpath_diff[{refpath_diff}]{blank}reflane_diff[{reflane_diff}]{blank}
        <br>
        </font>
        <font color=blue size={size}><b>
        priority[{priority}]{blank}prefer[{prefer_lane_cost}]{blank}
        <br>
        </font>
        <font color=gray size={size}><b>
        has_collision[{has_collision}]{blank}is_squeeze_in[{is_squeeze_in}]{blank}lc_task_type[{lc_task_type}]{blank}
        <br>
        nudge_safety[{nudge_safety}]{blank}lc_safety[{lc_safety}]{blank}lc_cost[{lc_task_cost}]{blank}\
        brake_correct[{brake_correct}]{blank}hold_time_cost[{hold_time_cost}]{blank}tag[{tag}]{blank}
        <br>
        <font color=red size={size}><b>
        simu:
        <br>
        </font>
        """.format(
            size = size,
            id = id,
            total_cost = get_cost('total_cost'),
            efficiency_cost = get_cost('efficiency_cost'),
            comfort_cost = get_cost('comfort_cost'),
            navigation_cost = get_cost('navigation_cost'),
            safety_cost = get_cost('safety_cost'),
            postprocess_cost = get_cost('post_cost'),

            v_end = get_cost('v_end'),
            v_front = get_cost('v_front'),
            s_end = get_cost ('s_end'),
            block_s = get_cost('block_s'),

            max_l = get_cost('max_l'),
            max_offset = get_cost('max_offset'),
            max_deltheta = get_cost('max_deltheta'),
            end_l = get_cost('end_l'),
            refpath_diff = get_cost('refpath_diff'),
            reflane_diff = get_cost('reflane_diff'),
            lat_acc = get_cost('lat_acc'),
            lon_acc = get_cost('lon acc'),
            lon_jerk = get_cost('lon_jerk'),
            refpath_dist = get_cost('refpath_distance'),
            max_kappa = get_cost('max_kappa'),
            max_expand = get_cost('max_expand'),

            priority = get_cost('navi_priority'),
            prefer_lane_cost = get_cost('prefer_lane_cost'),
            is_squeeze_in = get_cost('is_squeeze_in'),
            lc_task_type = get_cost('lc_task_type'),
            has_collision = get_cost('has_collision'),
            nudge_safety = get_cost('nudge_safety'),
            lc_safety = get_cost('lc_safety'),
            brake_correct = get_cost('brake_correct'),
            lc_task_cost = get_cost('lc_task_cost'),
            hold_time_cost = get_cost('hold_time_cost'),
            tag = get_cost('tag'),

            blank = "&nbsp;&nbsp;&nbsp;",
        )
        def add_mark(txt):
            return (',' if txt else '') + txt
        for sub_id, (s_cost, decision, source, sub_decision) in sorted(cost["simu_result"].items(), key = lambda e:e[0], reverse=True):
            text += """
            <font color=red size={size}><b>
            {sub_id}[{s_cost}{decision}{source}{sub_decision}]{blank}
            </font>
            """.format(
                size = size,
                sub_id = sub_id,
                s_cost = s_cost,
                decision = add_mark(decision),
                source = add_mark(source),
                sub_decision = add_mark(sub_decision),
                blank = "&nbsp;"
            )
        text += "</p>"
    return text

if __name__ == '__main__':
    start = time.perf_counter()
    end = time.perf_counter()
    print(end - start)
    load_all(os.path.dirname(__file__) + "/mm.txt")
    end = time.perf_counter()
    print(end - start)