"""Ti60 desktop controller. Serial operations run on one background worker."""
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText
from protocol import command, parse_hex, snapshot, query, MODES, LAYOUTS, VIEWS, SMOOTH

class SerialWorker(threading.Thread):
    def __init__(self, events):
        super().__init__(daemon=True)
        self.jobs = queue.Queue()
        self.events = events
        self.port = None
        self.motor_supported = False
    def emit(self, kind, value):
        self.events.put((kind,value))
    def log(self, value):
        self.emit('log',value)
    def run(self):
        while True:
            kind, arg = self.jobs.get()
            try:
                if kind == 'close':
                    if self.port: self.port.close()
                    self.port = None
                    self.emit('closed',None)
                elif kind == 'connect':
                    import serial
                    self.port = serial.Serial(arg,460800,timeout=.15,write_timeout=1)
                    self.port.reset_input_buffer()
                    result = snapshot(self.port,self.log)
                    self.motor_supported=result['motor_supported']
                    self.emit('connected',result)
                else:
                    if not self.port or not self.port.is_open:
                        raise RuntimeError('请先连接串口')
                    self.port.reset_input_buffer()
                    if kind == 'send':
                        if any(f[0] in (0x55,0x43) for f in arg) and not self.motor_supported:
                            raise RuntimeError('当前固件不支持串口电机控制，请先更新 FPGA 位流')
                        for frame in arg:
                            if len(frame)==1:
                                reply=query(self.port,frame[0])
                                self.log(f'TX {frame.hex(" ").upper()}  RX {reply.hex(" ").upper()}')
                            else:
                                self.port.write(frame)
                                self.log('TX '+frame.hex(' ').upper())
                        time.sleep(.15)
                    result=snapshot(self.port,self.log)
                    self.motor_supported=result['motor_supported']
                    self.emit('status',(result,kind=='sync'))
            except Exception as exc:
                if kind == 'connect':
                    if self.port: self.port.close()
                    self.port=None
                self.emit('error',str(exc))
            finally:
                self.emit('done',None)

class App:
    def __init__(self, root):
        self.root=root
        root.title('Ti60 图像处理控制台')
        root.geometry('1030x820');root.minsize(900,720)
        style=ttk.Style(root)
        if 'vista' in style.theme_names(): style.theme_use('vista')
        style.configure('Title.TLabel',font=('Microsoft YaHei UI',17,'bold'))
        style.configure('TButton',padding=(10,5))
        self.events=queue.Queue();self.worker=SerialWorker(self.events);self.worker.start()
        self.connected=False;self.pending=False
        self.motor_supported=False;self.motor_can_resume=False
        self.port=tk.StringVar(value='COM6')
        self.state=tk.StringVar(value='未连接')
        self.auto=tk.BooleanVar(value=False)
        self.annotation=tk.BooleanVar(value=True)
        self.values={}
        self.status=tk.StringVar(value='连接后读取板卡状态；不会自动更改参数')
        body=ttk.Frame(root,padding=16);body.pack(fill='both',expand=True)
        ttk.Label(body,text='Ti60 图像处理控制台',style='Title.TLabel').pack(anchor='w')
        ttk.Label(body,text='460800 · 8N1   |   DDR 帧控制、Canny 与亚像素边缘显示').pack(anchor='w',pady=(3,12))
        bar=ttk.Frame(body);bar.pack(fill='x')
        ttk.Label(bar,text='串口').pack(side='left')
        self.port_box=ttk.Combobox(bar,textvariable=self.port,width=13);self.port_box.pack(side='left',padx=8)
        ttk.Button(bar,text='刷新',command=self.refresh_ports).pack(side='left')
        self.connect_button=ttk.Button(bar,text='连接',command=self.toggle);self.connect_button.pack(side='left',padx=8)
        ttk.Label(bar,textvariable=self.state).pack(side='left',padx=8)
        self.actions=ttk.Frame(body);self.actions.pack(fill='x',pady=12)
        image=ttk.LabelFrame(self.actions,text='显示与边缘',padding=12);image.grid(row=0,column=0,sticky='nsew',padx=(0,8))
        frames=ttk.LabelFrame(self.actions,text='DDR 帧缓存',padding=12);frames.grid(row=0,column=1,sticky='nsew')
        self.actions.columnconfigure(0,weight=1);self.actions.columnconfigure(1,weight=1)
        self.combo(image,'layout','显示布局',LAYOUTS,3,0)
        self.combo(image,'view','算法视图',VIEWS,0,1)
        self.combo(image,'smooth','空间平滑',SMOOTH,0,2)
        self.number(image,'threshold','Canny 强阈值',12,4,32,3)
        self.number(image,'gain_r','红增益（÷64）',112,0,255,4)
        self.number(image,'gain_b','蓝增益（÷64）',96,0,255,5)
        ttk.Checkbutton(image,text='图形框与中文标注（最近发送值）',variable=self.annotation).grid(row=6,column=0,columnspan=2,sticky='w',pady=6)
        ttk.Button(image,text='应用图像设置',command=self.apply_image).grid(row=7,column=0,columnspan=2,sticky='ew')
        ttk.Label(image,text='叠加布局固定使用边缘；低覆盖度混合黄色。',wraplength=400).grid(row=8,column=0,columnspan=2,sticky='w',pady=6)
        self.number(frames,'age','A 帧年龄',0,0,15,0)
        self.number(frames,'reference','B 帧年龄',1,0,15,1)
        self.number(frames,'divider','回放保持 HDMI 帧数',2,1,255,2)
        ttk.Button(frames,text='应用选帧 / 回放间隔',command=self.apply_frames).grid(row=3,column=0,columnspan=2,sticky='ew',pady=6)
        modes=ttk.Frame(frames);modes.grid(row=4,column=0,columnspan=2,sticky='ew')
        for i,name in enumerate(MODES):
            ttk.Button(modes,text=name,command=lambda m=i:self.set_mode(m)).grid(row=i//3,column=i%3,sticky='ew',padx=2,pady=3)
        ttk.Label(frames,text='年龄 0 为最新完整帧；冻结后缓存不再更新。\n对比 / 差分按钮会同时切到全屏原图路径。\n返回实时后可重新应用左灰度＋黄色叠加布局。',wraplength=400).grid(row=5,column=0,columnspan=2,sticky='w',pady=10)
        motor=ttk.LabelFrame(self.actions,text='电机跟踪',padding=10)
        motor.grid(row=1,column=0,columnspan=2,sticky='ew',pady=(10,0))
        self.motor_text=tk.StringVar(value='连接后检查固件是否支持电机控制')
        self.pause_motor=ttk.Button(motor,text='暂停跟踪',command=lambda:self.set_motor(False))
        self.pause_motor.pack(side='left')
        self.resume_motor=ttk.Button(motor,text='恢复跟踪',command=lambda:self.set_motor(True))
        self.resume_motor.pack(side='left',padx=8)
        ttk.Label(motor,textvariable=self.motor_text).pack(side='left',padx=8)
        status=ttk.LabelFrame(body,text='板卡回读' ,padding=10);status.pack(fill='x')
        ttk.Label(status,textvariable=self.status,wraplength=940,justify='left').pack(anchor='w')
        controls=ttk.Frame(status);controls.pack(fill='x',pady=(8,0))
        ttk.Button(controls,text='读取并同步参数',command=lambda:self.submit('sync',None)).pack(side='left')
        ttk.Checkbutton(controls,text='每秒刷新状态',variable=self.auto).pack(side='left',padx=12)
        raw=ttk.Frame(body);raw.pack(fill='x',pady=10)
        self.hex_value=tk.StringVar(value='4D 03 52')
        ttk.Label(raw,text='HEX').pack(side='left')
        ttk.Entry(raw,textvariable=self.hex_value).pack(side='left',fill='x',expand=True,padx=8)
        ttk.Button(raw,text='发送',command=self.send_hex).pack(side='left')
        logbar=ttk.Frame(body);logbar.pack(fill='x')
        ttk.Label(logbar,text='通信记录').pack(side='left')
        ttk.Button(logbar,text='保存记录',command=self.save_log).pack(side='right')
        ttk.Button(logbar,text='清空',command=lambda:self.logbox.delete('1.0','end')).pack(side='right',padx=6)
        self.logbox=ScrolledText(body,height=11,font=('Consolas',10));self.logbox.pack(fill='both',expand=True,pady=(5,0))
        self.refresh_ports();self.enable_actions(False)
        root.protocol('WM_DELETE_WINDOW',self.quit)
        root.after(70,self.drain);root.after(1000,self.poll)
    def combo(self,parent,key,label,choices,index,row):
        ttk.Label(parent,text=label).grid(row=row,column=0,sticky='w',pady=5)
        var=tk.StringVar(value=choices[index]);self.values[key]=var
        ttk.Combobox(parent,textvariable=var,values=choices,state='readonly',width=32).grid(row=row,column=1,sticky='ew',padx=(10,0))
    def number(self,parent,key,label,value,low,high,row):
        ttk.Label(parent,text=label).grid(row=row,column=0,sticky='w',pady=5)
        var=tk.StringVar(value=str(value));self.values[key]=var
        ttk.Spinbox(parent,from_=low,to=high,textvariable=var,width=10).grid(row=row,column=1,sticky='ew',padx=(10,0))
    def enable_actions(self,enabled):
        def visit(widget):
            for child in widget.winfo_children():
                if isinstance(child,(ttk.Button,ttk.Spinbox,ttk.Combobox,ttk.Checkbutton)):
                    child.state(['!disabled'] if enabled else ['disabled'])
                visit(child)
        visit(self.actions)
        self.pause_motor.state(['!disabled'] if enabled and self.motor_supported else ['disabled'])
        self.resume_motor.state(['!disabled'] if enabled and self.motor_can_resume else ['disabled'])
    def refresh_ports(self):
        from serial.tools import list_ports
        self.port_box['values']=[p.device for p in list_ports.comports()]
    def toggle(self):
        if self.pending:return
        if self.connected:self.submit('close',None,require=False)
        else:self.submit('connect',self.port.get().strip(),require=False)
    def submit(self,kind,arg,require=True):
        if self.pending:return
        if require and not self.connected:
            messagebox.showinfo('未连接','请先选择串口并连接');return
        self.pending=True;self.connect_button.state(['disabled']);self.port_box.state(['disabled']);self.enable_actions(False)
        self.worker.jobs.put((kind,arg))
    def number_value(self,key,low,high):
        try:value=int(self.values[key].get(),10)
        except ValueError:raise ValueError('请输入十进制整数')
        if not low<=value<=high:raise ValueError(f'{key} 必须在 {low}～{high} 之间')
        return value
    def frame_commands(self):
        return [command(code,self.number_value(key,low,high)) for key,code,low,high in [('age',0x4A,0,15),('reference',0x4B,0,15),('divider',0x4F,1,255)]]
    def safe(self,fn):
        try:fn()
        except (ValueError,IndexError) as exc:messagebox.showerror('参数无效',str(exc))
    def apply_image(self):
        def send():
            frames=[command(0x4D,LAYOUTS.index(self.values['layout'].get())),command(0x56,[0,3][VIEWS.index(self.values['view'].get())]),command(0x53,SMOOTH.index(self.values['smooth'].get()))]
            frames += [command(code,self.number_value(key,4 if key=='threshold' else 0,32 if key=='threshold' else 255)) for key,code in [('threshold',0x54),('gain_r',0x41),('gain_b',0x42)]]
            frames.append(command(0x45,int(self.annotation.get())))
            self.submit('send',frames)
        self.safe(send)
    def apply_frames(self):self.safe(lambda:self.submit('send',self.frame_commands()))
    def set_mode(self,mode):
        def send():
            frames=self.frame_commands()
            if mode in (3,4):frames.append(command(0x4D,0))
            frames.append(command(0x50,mode));self.submit('send',frames)
        self.safe(send)
    def set_motor(self,enabled):
        if not self.motor_supported:
            messagebox.showinfo('固件不支持','请先生成并烧录支持串口电机控制的新固件');return
        self.safe(lambda:self.submit('send',[command(0x55,int(enabled))]))
    def send_hex(self):self.safe(lambda:self.submit('send',parse_hex(self.hex_value.get())))
    def show_status(self,s,sync=False):
        self.motor_supported=s.get('motor_supported',False)
        self.motor_can_resume=self.motor_supported and s['active_mode']==0 and not s.get('motor_blocked',True)
        if not self.motor_supported:self.motor_text.set('固件不支持：需更新 FPGA 位流')
        else:
            if s['motor_blocked'] or s['active_mode']!=0:label='暂停（历史帧联锁）'
            elif not s['motor_enabled']:label='已暂停'
            elif s['motor_rearming']:label='重新确认目标'
            elif not s['motor_status']&1:label='等待有效目标'
            elif s['motor_status']&8:label='脉宽命令限位'
            elif s['motor_status']&2:label='目标居中'
            else:label='正在跟踪'
            self.motor_text.set(label+'；K2 与串口共用状态')
        active=MODES[s['active_mode']] if s['active_mode']<len(MODES) else str(s['active_mode'])
        errors=s['errors']
        error_names=[name for bit,name in [(0,'采集不完整'),(1,'DDR 写错误'),(2,'DDR 读错误')] if errors&(1<<bit)]
        self.status.set(f"当前模式：{active}    源帧：{s['source']}    缓存：{s['retained']} 帧    A/B 年龄：{s['age']}/{s['reference']}    错误：0x{errors:02X} {'、'.join(error_names)}\n布局：{LAYOUTS[s['layout']]}    阈值：{s['threshold']}    平滑：{SMOOTH[s['smooth']]}    回放间隔：{s['divider']} 个 HDMI 帧")
        if sync:
            for key in ['age','reference','divider','threshold','gain_r','gain_b']:self.values[key].set(str(s[key]))
            self.values['layout'].set(LAYOUTS[s['layout']]);self.values['smooth'].set(SMOOTH[s['smooth']]);self.values['view'].set('灰度' if s['view']==3 else '边缘')
    def drain(self):
        while True:
            try:kind,arg=self.events.get_nowait()
            except queue.Empty:break
            if kind=='log':self.logbox.insert('end',time.strftime('%H:%M:%S')+'  '+arg+'\n');self.logbox.see('end')
            elif kind=='connected':self.connected=True;self.state.set('已连接 '+self.port.get());self.connect_button.configure(text='断开');self.show_status(arg,True)
            elif kind=='closed':
                self.connected=False;self.motor_supported=False;self.motor_can_resume=False
                self.motor_text.set('未连接');self.state.set('未连接');self.connect_button.configure(text='连接')
            elif kind=='status':self.show_status(*arg)
            elif kind=='error':
                self.logbox.insert('end','错误：'+arg+'\n');self.state.set('通信错误')
                messagebox.showerror('通信失败',arg+'\n若提示拒绝访问，请关闭占用串口的其他软件。')
            elif kind=='done':
                self.pending=False;self.connect_button.state(['!disabled']);self.port_box.state(['disabled'] if self.connected else ['!disabled']);self.enable_actions(self.connected)
        if int(self.logbox.index('end-1c').split('.')[0]) > 1200:
            self.logbox.delete('1.0','201.0')
        self.root.after(70,self.drain)
    def poll(self):
        if self.auto.get() and self.connected and not self.pending:self.submit('poll',None)
        self.root.after(1000,self.poll)
    def save_log(self):
        path=filedialog.asksaveasfilename(defaultextension='.txt',filetypes=[('文本记录','*.txt')])
        if path:
            with open(path,'w',encoding='utf-8') as f:f.write(self.logbox.get('1.0','end'))
    def quit(self):
        self.worker.jobs.put(('close',None));self.root.destroy()

if __name__=='__main__':
    root=tk.Tk();App(root);root.mainloop()
