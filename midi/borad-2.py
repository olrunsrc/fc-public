from pygame import midi
from time import sleep
import yaml

global theSmartie
theSmartie = None

class Mbase(yaml.YAMLObject):
    def __init__(self,row,col,key):
        self.row = row
        self.col = col
        self.key = key
	self.lupd = 0
	self.val = 64

    def __repr__(self):
	return "!%s %s (%d,%d) = %d" % (self.__class__.__name__,
		self.key,self.row,self.col,self.val)

        
class Mctl(Mbase):
    global theSmartie
    yaml_tag = "!Mctl"
    def __init__(self,parent,row,col,key,**opts):
        super(Mctl,self).__init__(row,col,key)
	self.parent = parent
        self.val = int(opts.get('val',64))
        self.min = float(opts.get('min',-1))
        self.max = float(opts.get('max',1))

    def update(self,miditstamp,newval):
	self.lupd = miditstamp
        self.val = int(newval)
        print "(%d,%d)<=%d" % (self.row,self.col,self.val)
        return self

    def get_rosval(self):
	rosval = ((self.max - self.min)/127.0)*self.val + self.min
	return rosval

    def get_row(self):
	return self.parent.ctl8x8[self.row]

    def get_sibling(self,name):
	#clist = filter(lambda c: c.key==name,self.parent.ctl8x8[self.row])
	clist = filter(lambda c: c.key==name,theSmartie.ctl8x8[self.row])
	print "Sibling %s" % clist
	if len(clist)==1 : return clist[0]
	return None
	
	
class Mnote(Mbase):
    yaml_tag = "!Mnote"
    def __init__(self,row,col):
        super(Mnote,self).__init__(row,col,' ')
        self.num = 0
        self.val = 0
        self.min = 0
        self.max = 127
        self.rosoffset = 0
        self.rosslope = 1
    def init(self): return True
    def noteon(self):
   	self.val = 127
        print "(%d,%d) On" % (self.row,self.col)
        return self
    def noteoff(self):
	self.val = 0
        print "(%d,%d) Off" % (self.row,self.col)
        return self

class Mbtn(Mbase):
    yaml_tag = "!Mbtn"
    def __init__(self,parent,row,col,key=' ',**opts):
        super(Mbtn,self).__init__(row,col,key)
	self.parent = parent
        self.val = bool(opts.get('val',False))
	self.toggle = bool(opts.get('toggle',False))
    def init(self): return True
    def save(self): return True
    def saveall(self): return True
    def clear(self): return True
    def select(self): return True
    def update(self): return True
    def noteon(self):
	if self.toggle: self.val = not self.val
	else: self.val = True
        print "(%d,%d) Press" % (self.row,self.col)
        return self
    def noteoff(self):
	if self.toggle: pass
	else: self.val = False
        print "(%d,%d) Release" % (self.row,self.col)
        return self

class SmartPAD():

    #controller_name = 'nanoKONTROL'
    controller_name = 'SmartPAD'

    def __init__(self,ctlname):

	global theSmartie
        theSmartie = self

        self.ctl8x8 = [] 
        self.btn8x9 = []

        midi.init()
        for i in range(midi.get_count()):
            (_,name,b_in,bout,_) = midi.get_device_info(i)
            if ctlname in str(name) and b_in:
                self.indev = midi.Input(i)
            if ctlname in str(name) and bout:
                self.outdev = midi.Output(i)
        self.device = midi.get_device_info(self.indev.device_id)[1]
        print "Midi device is %s" % self.device

        self.default_btns(self.ctl8x8, self.btn8x9)

    def finish(self):
	pass
        #midi.quit()
        
    def readin(self):
        done = False
        data = []
        mi, mo = self.indev, self.outdev
        while mi.poll():
            msg,tstamp = mi.read(1)[0]
            #print "Msg %s at time %d" % (msg,tstamp)
            cmd = (msg[0] & 0xF0) >> 4
            chan = msg[0] & 0x0F
            row = (msg[1] & 0xF0) >> 4
            col = msg[1] & 0x0F
            msgt = (cmd,chan,row,col)
            #print "Cmd %x Chan %x Row %x Col %x" % msgt
            if msgt == (0x8,2,7,7): print "Done"
            if msg[0]==130 and msg[1]==119:
                done = True
                break
            if msg[0]==176:
                data.append(self.ctl8x8[row][col].update(tstamp,msg[2]))
                #break
            if msg[0]==144:
                data.append(self.btn8x9[row][col].noteon())
                #break
            if msg[0]==128:
                data.append(self.btn8x9[row][col].noteoff())
                #break
            if msg[0]==146:
                data.append(self.btn8x9[col][8].noteon())
                #break
            if msg[0]==130:
                data.append(self.btn8x9[col][8].noteoff())
                #break
        return done, data

    def loop(self):
        done, data = self.readin()
        while not done:
            sleep(0.01)
            done, data = self.readin()

    def default_btns(self,ctl8x8,btn8x9):
        for i in range(7): #rows
            ctl8x8.append([])
            btn8x9.append([])
            for j in range(8): #cols
                ctl8x8[i].append( Mctl(self,i,j, 'm') )
                btn8x9[i].append( Mnote(i,j) )
	    btn8x9[i].append( Mbtn(self,i,8) )
        ctl8x8.append([])
        btn8x9.append([]) 
        for j in range(8): #cols
            ctl8x8[7].append( Mctl(ctl8x8,7,j, 'y') )
            btn8x9[7].append( Mbtn(self,7,j) )
        btn8x9[7].append( Mbtn(self,7,8) )
        
	#top row - left arm
        ctl8x8[0][0] = Mctl(self,0,0,'la0',min=-3.0,max=3.0)
        ctl8x8[0][1] = Mctl(self,0,1,'la1',min=-3.0,max=3.0)
        ctl8x8[0][2] = Mctl(self,0,2,'la2',min=-3.0,max=3.0)
	ctl8x8[0][3] = Mctl(self,0,3,'la3',min=-2.0,max=0.1)
        ctl8x8[0][4] = Mctl(self,0,4,'la4',min=-2.0,max=3.0)
        ctl8x8[0][5] = Mctl(self,0,5,'la5',min=-0.6,max=0.6)
        ctl8x8[0][6] = Mctl(self,0,6,'la6',min=-0.3,max=0.4)
        ctl8x8[0][7] = Mctl(self,0,7,'tim',min=0.01,max=2.0)
	
	#2nd row - right arm
        ctl8x8[1][0] = Mctl(self,1,0,'ra0',min=-3.0,max=3.0)
        ctl8x8[1][1] = Mctl(self,1,1,'ra1',min=-3.0,max=3.0)
        ctl8x8[1][2] = Mctl(self,1,2,'ra2',min=-3.0,max=3.0)
	ctl8x8[1][3] = Mctl(self,1,3,'ra3',min=-0.1,max=2.0)
        ctl8x8[1][4] = Mctl(self,1,4,'ra4',min=-2.0,max=3.0)
        ctl8x8[1][5] = Mctl(self,1,5,'ra5',min=-0.6,max=0.6)
        ctl8x8[1][6] = Mctl(self,1,6,'ra6',min=-0.4,max=0.3)
        ctl8x8[1][7] = Mctl(self,1,7,'tim',min=0.01,max=2.0)

	#3rd row - left and right hands
        ctl8x8[2][0] = Mctl(self,2,0,'lh0',min=0.0,max=2.3)
        ctl8x8[2][1] = Mctl(self,2,1,'lht',min=-0.9,max=0.0)
        ctl8x8[2][2] = Mctl(self,2,2,'lhi',min=-1.0,max=0.0)
	ctl8x8[2][3] = Mctl(self,2,3,'lhp',min=-1.0,max=0.0)
        ctl8x8[2][4] = Mctl(self,2,4,'rh0',min=0.0,max=2.3)
        ctl8x8[2][5] = Mctl(self,2,5,'rht',min=0.0,max=0.9)
        ctl8x8[2][6] = Mctl(self,2,6,'rhi',min=0.0,max=1.0)
        ctl8x8[2][7] = Mctl(self,2,7,'rhp',min=0.0,max=1.0)

	#4th row - chest rpy, pyp, pelvis ht, tim
        ctl8x8[3][0] = Mctl(self,3,0,'chr',min=-3.0,max=3.0)
        ctl8x8[3][1] = Mctl(self,3,1,'chp',min=-3.0,max=3.0)
        ctl8x8[3][2] = Mctl(self,3,2,'chy',min=-3.0,max=3.0)
	ctl8x8[3][3] = Mctl(self,3,3,'npl',min=-0.1,max=2.0)
        ctl8x8[3][4] = Mctl(self,3,4,'nym',min=-1.0,max=1.0)
        ctl8x8[3][5] = Mctl(self,3,5,'npu',min=-0.6,max=0.6)
        ctl8x8[3][6] = Mctl(self,3,6,'',min=-0.4,max=0.3)
        ctl8x8[3][7] = Mctl(self,3,7,'tim',min=0.01,max=2.0)


	btn8x9[7][2] = Mbtn(self,7,2,'lar')
	btn8x9[7][3] = Mbtn(self,7,2,'rar')


if __name__ == '__main__':
    sp = SmartPAD('SmartPAD')
    sp.loop()
    sp.finish()
    

