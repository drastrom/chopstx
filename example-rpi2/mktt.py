# Make a translation table from the configuration.
# Currently 1 stage tt only.
import sys

class TTable:
    """
    Translation table.
    """
    MANAGER=0
    CLIENT=1
    NA=2
    def section(self,pa=0,ns=0,ng=0,s=0,ap=3,tex=0,
                dom=MANAGER,xn=0,c=0,b=0,valid=1,pxn=0):
        """
        Make a section entry.
        """
        return ((pa << 20)|(ns << 19)|(ng << 17)|(s << 16)
                |((ap >> 2) << 15)|(tex << 12)|((ap & 3) << 10)
                |(dom << 5)|(xn << 4)|(c << 3)|(b << 2)
                |(valid << 1)|(pxn << 0))
    def __init__(self):
        self.size=4096
        self.name='default_translation_table'
        self.secname='.translation.descriptors'
        self.trans=[self.section(pa,valid=0) for pa in range(self.size)]
        return
    def section_update(self,pa,ns=0,ng=0,s=0,ap=3,tex=0,
                dom=MANAGER,xn=0,c=0,b=0,valid=1,pxn=0):
        self.trans[pa] = self.section(pa,ns,ng,s,ap,tex,dom,xn,c,b,valid,pxn)
        return
    def range_update(self,pa,size,ns=0,ng=0,s=0,ap=3,tex=0,
                dom=MANAGER,xn=0,c=0,b=0,valid=1,pxn=0):
        for a in range(pa,pa+size):
            self.section_update(a,ns,ng,s,ap,tex,dom,xn,c,b,valid,pxn)
        return
    def output(self):
        attrib='__attribute__ ((section("%s"),aligned(0x4000)))'%self.secname
        print('unsigned long %s[%d] %s'%(self.name, self.size, attrib))
        print('= {')
        for a in range(self.size):
            print('0x%08x,'%self.trans[a])
        print('};')
        return

tt=TTable()
tt.range_update(0,size=1024-64,s=1,c=1,b=1)
tt.range_update(0x3f0,size=16,s=1,b=1)
tt.range_update(0x400,size=1,s=1,b=1)
tt.output()
