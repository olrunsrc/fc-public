import inspect,re,os

from POSH.utils import get_root_path

class Profiler:
    """This class was written by Philipp Rohlfshagen to report the data found in Rohlfshagen & Bryson (2008).
    It should only be included if profiling is desirable.  For an example of its use, see library latchTest and 
    the experiment script replication-scripts/profiled_experiment_executer.py
    
    Note that this file is not currently well-written, but has a great deal of special-purpose code in it.
    For example, a data directory is hardcoded below.  This script does not create the directory; this must all be set up.

    You will want to save results you publish somewhere archival after the run.
    
    If you want to use this, you need to turn on profiling in agent_base.py before loading the behaviours.  Do this by 
        POSH.profiler.Profiler.turnOnProfiling()
    Comment by JJB, 29 Feb 2008 (updated 2 April)
    """
    
    LIMIT=5001
    directory=get_root_path()+'/replication-scripts/data/'
    
    def __init__(self,name):
        self._name=name
        self._logger={}
        self._associations={}  
        
        self._counts={}
        self._avgs={}      

        self.total_calls=0
        
    # this normally does nothing.  If you want it to do something, you have to reset it!  
    # (see next two functions)  
    def initProfile(myclass,other):
        return None
    
    def _reallyInitProfile(myclass,other):     
        other.profiler = Profiler(other.id)
        return other.profiler

    # call this in your init_world to turn on profiling.  This changes what POSH.agent_base.AgentBase effectively does.
    # this is a class method and needs no instance
    def turnOnProfiling():
        Profiler.initProfile = Profiler._reallyInitProfile
    turnOnProfiling = staticmethod(turnOnProfiling)
    _reallyInitProfile = classmethod(_reallyInitProfile)
    initProfile = classmethod(initProfile)
    
    def set_second_name(self,second_name):
        self._second_name=second_name
        
    def register(self,klass,names):
        if names==():
            names=tuple(self.get_all_functions(klass))

        for name in names:
            self._logger[name]=0
            self._associations[name]=klass.__class__.__name__
            setattr(klass,name,self.wrapper(name,getattr(klass,name)))
              
    def wrapper(self,name,fun):
        def new_fun(*args, **kwargs):
            self._logger[name]+=1
            return fun(*args, **kwargs)
        return new_fun                    
        
    def get_all_functions(self,klass):    
        all_defs=inspect.getmembers(klass,inspect.isroutine)
        all_methods=[]
    
        s=re.compile('^s_[.]*')
        a=re.compile('^a_[.]*')
    
        for (name,definition) in all_defs:
            if s.match(name) or a.match(name):
                all_methods.append(name)    
    
        return all_methods
        
    def increase_total_calls(self,num_calls=1):
        self.total_calls+=num_calls

        if self.total_calls==self.LIMIT:
            file_name='data/agent_%s_log.txt' % (self._name)
            self.write_to_file(file_name)
        
    def reset(self):
        self._logger={}
        self._associations={}
        self._name=None
        
    def set_info(self,info):
        self._info=info        
        
    def get_name(self):
        return self._name
        
    def get_num_calls(self,name):
        return self._logger[name]
    
    def get_registered_methods(self):
        return self._logger.keys()
    
    def get_registered_associations(self):
        return self._associations.values()
    
    def get_logger(self):
        return self._logger
    
    def get_associations(self):
        return self._associations
    
    def get_formatted_output(self,dict):
        str=''
        
        keys=dict.keys()
        keys.sort()

        for key in keys:
            str+='>>\t%s\t%f\n' % (key, dict[key])
        
        return str    
    
    def write_to_file(self,output_file):
        if not os.path.exists(output_file):        
            log=open(output_file,'w')
            print >> log,self._logger.keys()

            if hasattr(self,'_info'):
                print >> log,self._info         
            else:
                print >> log,'no information supplied'       
        else:
            log=open(output_file,'a')
                    
        if hasattr(self,'_second_name'):
            print >> log,self._name,self._second_name
        else:
            print >> log,self._name
        
        print >> log,self.total_calls
        print >> log,self.get_formatted_output(self._logger)         
        log.close()   
        
    def load_file(self,file_name):
        file=open(file_name,'r')
    
        for line in file.readlines():                        
            columns=line.split()
            
            if len(columns)==3 and columns[0]=='>>':
                if self._avgs.has_key(columns[1]):                
                    self._avgs[columns[1]]+=float(columns[2])
                    self._counts[columns[1]]+=1
                else:
                    self._avgs[columns[1]]=float(columns[2])
                    self._counts[columns[1]]=1
                
    def compute_average(self,file_name):
        self.load_file(file_name)
        items=self._avgs.keys()
        
        print items
        
        for item in items:
            self._avgs[item]/=self._counts[item]
            
        print self._avgs      
        
        log=open(file_name,'a')
        print >> log,'AVERAGES'
        print >> log,self.get_formatted_output(self._avgs)         
        log.close()
        
    def compute_multi_agent_avg(self,target_file,agents):    
        avgs={}
        counts={}
        
        for agent in agents:
            file_name=self.directory+'/agent_%s_log.txt' % (agent)
            file=open(file_name,'r')
        
            for line in file.readlines():                        
                columns=line.split()
                
                if len(columns)==3 and columns[0]=='>>':
                    if avgs.has_key(columns[1]):                
                        avgs[columns[1]]+=float(columns[2])
                        counts[columns[1]]+=1
                    else:
                        avgs[columns[1]]=float(columns[2])
                        counts[columns[1]]=1            
            
        items=avgs.keys()

        for item in items:
            avgs[item]/=counts[item]
                
        log=open(target_file,'a')
        print >> log,'AVERAGES'
        print >> log,self.get_formatted_output(avgs)         
        log.close()
   
    def compute_depletion_avg(self,target_file,agents):    
        avgs={}
        counts={}
        
        for agent in agents:
            file_name=self.directory+'/agent_%s_log.txt' % (agent)
            file=open(file_name,'r')
        
            for line in file.readlines():                        
                columns=line.split()
                
                if len(columns)==3 and columns[0]=='>>' and columns[1]=='depleted':
                    if avgs.has_key(columns[1]):                
                        avgs[columns[1]]+=float(columns[2])
                        counts[columns[1]]+=1
                    else:
                        avgs[columns[1]]=float(columns[2])
                        counts[columns[1]]=1            
            
        items=avgs.keys()

        for item in items:
            avgs[item]/=counts[item]
                
        log=open(target_file,'a')
        print >> log,self.get_formatted_output(avgs)         
        log.close()
      
if __name__=='__main__':     
    p=Profiler(None)
    p.compute_multi_agent_avg(p.directory+'final_result.txt',['A00','A01','A02','A03','A04'])
    p.compute_depletion_avg(p.directory+'final_result.txt',['A05','A06','A07','A08'])