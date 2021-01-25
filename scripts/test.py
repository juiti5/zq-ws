 
import Tkinter
 
root_window = Tkinter.Tk()
root_window.title('Tkinter_Demo')
root_window.geometry('400x300')
 
m_text = Tkinter.Text(root_window)
m_text.insert(Tkinter.CURRENT, 'hello \n')
m_text.insert(Tkinter.END, 'world \n')
m_text.insert(Tkinter.END, 'nono')
m_text.pack()
root_window.mainloop()
