import tkinter as tk

acceptedNum = "-0123456789."  # for requiring number input
def decimal_val(val:tk.StringVar,_,__,___):
    perFlag=0
    tempStr=val.get()
    for i in range(1,len(tempStr)):
        if tempStr[i]=='-':
            tempStr=tempStr[:i]+tempStr[i+1:]
            break
    for i in tempStr:
        if i not in acceptedNum:
            tempStr=tempStr.replace(i, "")
    if tempStr.count('.')>0:
        for i in range(len(tempStr)):
            if tempStr[i]=='.' and perFlag==0:
                perFlag=1
            elif tempStr[i]=='.':
                tempStr=tempStr[:i]+tempStr[i+1:]
                break
    val.set(tempStr)