import numpy                                    as np
import matplotlib.pyplot                        as plt
from dynamic_graph                              import writeGraph
from sot_talos_balance.create_entities_utils    import addTrace, create_tracer, dump_tracer
from dynamic_graph.tracer_real_time             import TracerRealTime
from time                                       import sleep
from IPython                                    import embed
import os

def read_tracer_file(filename):
    data = np.loadtxt(filename);
    name = filename[8:-4]
    return data, name

def plot_select_traj(traj,idxs,name):
    ''' plot selected idx of ND array'''
    plt.figure()
    nb_plots = np.size(idxs)
    for idx in idxs:
        plt.plot(traj[:,idx])
        plt.title(name)

def write_pdf_graph(path):
    ''' outputs a pdf of the graph to the specified path '''
    writeGraph(path+'graph.dot')
    os.system('dot -Tpdf '+path+'graph.dot -o '+path+'graph.pdf')
    return
    
def dump_sot_sig(robot,entity,signal_name,duration):
    '''dumps a sot signal in /tmp
    ex: dump_sot_sig(robot,robot.entity,'signal_name',1.)'''
    full_sig_name          = entity.name +'.'+signal_name
    robot.tmp_tracer  = create_tracer(robot,entity,'tmp_tracer', [signal_name])
    robot.device.after.addSignal(full_sig_name)
    robot.tmp_tracer.start()
    sleep(duration)
    dump_tracer(robot.tmp_tracer)
    robot.tmp_tracer.clear()

def dump_sot_sigs(robot,list_of_sigs,duration):
    '''dumps several sot signals in /tmp
    ex: dump_sot_sig(robot,[entity,signals],1.)'''
    tracer = TracerRealTime('tmp_tracer')
    tracer.setBufferSize(80*(2**20))
    tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(tracer.name))
    for sigs in list_of_sigs:
            entity = sigs[0]
            for sig in sigs[1:]:
                full_sig_name = entity.name +'.'+sig
                addTrace(tracer,entity,sig)
                robot.device.after.addSignal(full_sig_name)
    tracer.start()
    sleep(duration)
    dump_tracer(tracer)
    tracer.clear()

def plot_sot_sig(filename,idxs):
    '''plots a dumped signal'''
    filename = '/tmp/dg_'+filename+'.dat'
    data, name = read_tracer_file(filename)
    plot_select_traj(data,idxs,name)
    return    
