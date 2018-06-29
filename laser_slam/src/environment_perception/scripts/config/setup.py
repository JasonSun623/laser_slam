import sys,os
import rospkg
rospack=rospkg.RosPack()

_PackPath=rospack.get_path('environment_perception')


names_path=os.path.join(_PackPath+'/scripts/config/','voc.names')
data_path=os.path.join(_PackPath+'/scripts/config/','voc.data')
with open(data_path,'r') as f:
     lines=f.readlines()
with open(data_path,'w') as f_w:
     for line in lines:
         if "names" in line:
             line="names="+names_path+'\n'
         f_w.write(line)
f.close
f_w.close


op_path=os.path.join(_PackPath+'/scripts/_darknet/','libdarknet.so')

net_path=os.path.join(_PackPath+'/scripts/config/','net.cfg')

model_path=os.path.join(_PackPath+'/scripts/config/','model.weights')

meta_path=os.path.join(_PackPath+'/scripts/config/','voc.data')

record_path=os.path.join(_PackPath+'/scripts/_record/')

yaml_path=os.path.join(_PackPath+'/scripts/config/','ost.yaml')
