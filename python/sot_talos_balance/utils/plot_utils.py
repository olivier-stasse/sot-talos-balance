import numpy as np
import matplotlib.pyplot as plt
from dynamic_graph import writeGraph
import os

def read_tracer_file(filename):
    data = np.loadtxt(filename);
    name = filename[8:-4]
    return data, name

def plot_select_traj(traj,idxs,name):
    ''' plot selected idx of ND array'''
    nb_plots = np.size(idxs)
    for idx in idxs:
        plt.plot(traj[:,idx])
        plt.title(name)
    plt.show()
    return

def write_pdf_graph(path):
    ''' outputs a pdf of the graph to the specified path '''
    writeGraph(path+'graph.dot')
    os.system('dot -Tpdf '+path+'graph.dot -o '+path+'graph.pdf')
    return
    
