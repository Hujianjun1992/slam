#!/usr/bin/python

import os
path="./"
for root,dirs,files in os.walk(path):
    for name in files:
        print name
        if name.endswith(".txt"):
            print root,dirs,name
            filename=root+"/"+name
            f=open(filename,"r")
            filecontent=""
            line=f.readline()
            while line:
               l=line.replace("nan","100")
               filecontent=filecontent+l
               line=f.readline()
            f.close()
            f2=file(filename,"w")
            f2.writelines(filecontent)
            f2.close()
