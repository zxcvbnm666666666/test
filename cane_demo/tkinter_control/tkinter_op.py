import tkinter as tk
import time

def tk_fun(q1, q2):
    root = tk.Tk()
    root.title('甘蔗信息管理系统 v1.0')
    root.geometry('600x400')

    menubar = tk.Menu(root)
    menubar.add_command(label="主界面")
    menubar.add_command(label="子界面")
    root.config(menu=menubar)

    label = tk.Label(root, text="排杂风机").place(x=20, y=40, anchor='nw')
    label = tk.Label(root, text="二级输送").place(x=20, y=60, anchor='nw')
    label = tk.Label(root, text="切断刀").place(x=20, y=80, anchor='nw')
    label = tk.Label(root, text="根切器").place(x=20, y=100, anchor='nw')
    var = tk.StringVar()

    while not q1.empty() or not q2.empty():
        if not q1.empty():
            a = q1.get()
            var.set(str(a))
            label = tk.Label(root, text=a).place(x=80, y=60, anchor='nw')
            # print('a',a)
            time.sleep(1)

        if not q2.empty():
            a1 = q2.get()
            var.set(str(a1))
            label = tk.Label(root, text=a1).place(x=80, y=80, anchor='nw')
            # print('a',a1)

        root.update()
    root.mainloop()
