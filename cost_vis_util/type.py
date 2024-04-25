class RefPath(object):
    def __init__(self):
        self.pts = []  # x, y, theta
        self.label = "default"

class SceneContent(object):
    def __init__(self, refpath):
        self.ego_param = [5., 2.1, 3.5, 1.5] # length, width, front edge to vrp, back edge to vrp
        self.ego_path = refpath # RefPath
        self.states_dict = {} # id: states
        self.param_dict = {} # id: param(length, width)
        self.costs_dict = {} # id: {cost_name: cost_value}
        self.max_frame = 0

    def text(self):
        context = self.ego_path.label + "\n"
        for id, costs in self.costs_dict.items():
            context += (id + ": ")
            for name, val in costs.items():
                context += (name + ":" + str(round(val, 2)) + "|")
            context += "\n"
        return context


