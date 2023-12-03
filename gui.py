import tkinter as tk
import tkinter.messagebox
from tkinter import ttk
import numpy as np
import simplex
from gilp.simplex import LP
from gilp.visualize import simplex_visual


class SimplexSolver:
  def __init__(self, c, A, b, sense):
    """
    Initializes the simplex solver.

    Args:
      c: A vector of coefficients for the objective function.
      A: A matrix of coefficients for the constraints.
      b: A vector of right-hand sides for the constraints.
      sense: A string indicating whether to maximize or minimize the objective function.
    """

    self.c = c
    self.A = A
    self.b = b
    self.sense = sense

  def solve(self):
    """
    Solves the linear programming problem.

    Returns:
      A tuple of the optimal solution and the optimal value of the objective function.
    """

    # Initialize the simplex tableau.
    tableau = np.zeros((len(self.c) + len(self.b) + 1, len(self.c) + len(self.b) + 1))
    for i in range(len(self.c)):
      tableau[i, i] = self.c[i]
    for i in range(len(self.b)):
      tableau[len(self.c) + i, i] = -1
    for i in range(len(self.b)):
      tableau[len(self.c) + len(self.b) + 1, len(self.c) + i] = self.b[i]

    # Iterate until the optimal solution is found.
    while True:
      # Find the pivot column.
      pivot_column = np.argmin(tableau[len(self.c) + len(self.b) + 1, :len(self.c)])

      # Find the pivot row.
      pivot_row = np.argmax(tableau[:len(self.c), pivot_column])

      # Perform the pivot operation.
      tableau[pivot_row, :] /= tableau[pivot_row, pivot_column]
      tableau[:, pivot_column] /= tableau[pivot_row, pivot_column]
      for i in range(len(self.c)):
        if i != pivot_row:
          tableau[i, pivot_column] *= -tableau[pivot_row, i]

      # Check for optimality.
      if np.all(tableau[len(self.c) + len(self.b) + 1, :len(self.c)] >= 0):
        break

    # Return the optimal solution.
    optimal_solution = np.zeros(len(self.c))
    for i in range(len(self.c)):
      optimal_solution[i] = tableau[i, len(self.c) + len(self.b) + 1]

    optimal_value = tableau[len(self.c) + len(self.b) + 1, len(self.c) + len(self.b) + 1]

    return optimal_solution, optimal_value


class App():
    def __init__(self):
        # parent frame
        self.root = tk.Tk()
        self.root.geometry('500x300')
        self.root.title('simplex app')
        self.root.resizable(1, 1)

        # vars
        self.A = []
        self.b = []
        self.c = []
        self.subject = []
        self.eq_constraint_list = []
        self.eq = []
        self.eqv = []

        # children frame
        self.mainframe = tk.Frame(self.root, background='white')
        self.mainframe.pack(fill='both', expand='True')

        # self.inputframe =tk.Frame(self.root, background='white')
        # self.inputframe.pack(fill='both', expand='True')

        # self.resultframe = tk.Frame(self.root, background='white')
        # self.resultframe.pack(fill='both', expand='True')

        self.mainFrame()
        self.root.mainloop()
        return

    # get a number of vars
    def get_nvars(self):
        # print(type(self.text_vars_field.get()))
        return self.text_vars_field.get()

    # get min or max
    def get_subject(self):
        return self.sub_field.get()

    # get a number of constraints
    def get_nconstraints(self):
        return self.text_contraints_field.get()

    # matrix A
    def getMatrixA(self):
        col = int(self.get_nvars())
        row = int(self.get_nconstraints())
        listA = []
        for i in range(row * col):
            listA.append(self.A[i].get())

        # matA = np.array(listA).reshape(row,col)
        # print(listA)
        return list(listA)

    # matrix b
    def getMatrixb(self):
        n = int(self.get_nconstraints())
        matb = []
        for i in range(n):
            matb.append(self.b[i].get())
            # print(self.b[i].get())
        return list(matb)

    # matrix c
    def getMatrixc(self):
        n = int(self.get_nvars())
        matc = []
        for i in range(n):
            matc.append(self.c[i].get())
            # print(self.c[i].get())
        # print(list(matc))
        return list(matc)

    def geteq_contraints(self):
        n = int(self.get_nconstraints())
        listeq = []
        for i in range(n):
            listeq.append(self.eq_constraint_list[i].get())
        # print(listeq)
        return listeq

    def geteq_contraint_vars(self):
        n = int(self.get_nvars())
        listeqvar = []
        for i in range(n):
            listeqvar.append(self.eq_constraint_var_list[i].get())
        # print(listeqvar)
        return listeqvar

    # remove everything on the frame
    def clear_frame(self, frame):
        for widgets in frame.winfo_children():
            widgets.destroy()

    def str2float(self, string_list):
        float_list = [float(element) for element in string_list]
        return float_list
    def check_input_frame(self):
        nvars = int(self.get_nvars())
        ncons = int(self.get_nconstraints())
        matc = self.getMatrixc()
        matb = self.getMatrixb()
        matA = self.getMatrixA()
        eq = self.geteq_contraints()
        eqv = self.geteq_contraint_vars()
        sub = self.get_subject()
        M = matc + matb + matA

        #print(matA)
        try:
            if all(isinstance(float(x), float) for x in M):
                self.inputframe.forget()
                self.input_result_frame(nvars, ncons, matA, matb, matc, eq, eqv,sub)
        except:
            mess = tkinter.messagebox.showerror(title='Error', message='vui long nhap day du cac he so')

    def check_main_frame(self):
        # get a number of vars and constraints
        nvars = self.get_nvars()
        ncons = self.get_nconstraints()
        try:
            if int(nvars) >= 2 and int(ncons) >= 1:
                self.mainframe.forget()
                self.main_input_frame()
            else:
                mess = tkinter.messagebox.showerror(title='Error', message='vui long nhap lai, so bien, so rang buoc khong dung')
        except:
            mess = tkinter.messagebox.showerror(title='Error', message='vui long nhap lai, so bien, so rang buoc khong dung')

    # main frame for inputing var, sub, cons
    def mainFrame(self):

        self.root.geometry('500x300')
        # grid
        self.mainframe.columnconfigure(0, weight=1)
        self.mainframe.columnconfigure(1, weight=1)
        self.mainframe.rowconfigure(0, weight=1)
        self.mainframe.rowconfigure(1, weight=1)
        self.mainframe.rowconfigure(2, weight=1)
        self.mainframe.rowconfigure(3, weight=1)

        # subject
        subject_option = ['min', 'max']
        self.text_sub = ttk.Label(self.mainframe, text='Which is the objective of the function?', justify='center',
                                  background='white', font=('Brass Mono', 10))
        self.text_sub.grid(row=0, column=0, sticky='E', padx=5)

        self.sub_field = ttk.Combobox(self.mainframe, values=subject_option, width=4, state='readonly')
        self.sub_field.grid(row=0, column=1, sticky=tk.W)
        self.sub_field.current(0)

        # number of variables
        self.text_vars = ttk.Label(self.mainframe, text='How many decision variables are the problem?',
                                   justify='center', background='white', font=('Brass Mono', 10))
        self.text_vars.grid(row=1, column=0, sticky=tk.E, padx=5)

        self.text_vars_field = ttk.Entry(self.mainframe, width=10)
        self.text_vars_field.grid(row=1, column=1, sticky=tk.W, padx=5)

        # number of constraints
        self.text_constraints = ttk.Label(self.mainframe, text='How many constraints?', justify='center',
                                          background='white', font=('Brass Mono', 10))
        self.text_constraints.grid(row=2, column=0, sticky=tk.E, padx=5)

        self.text_contraints_field = ttk.Entry(self.mainframe, width=10)
        self.text_contraints_field.grid(row=2, column=1, sticky=tk.W, padx=5)

        # continue button
        # self.btn_continue = ttk.Button(self.mainframe, text='continue',command= self.main_input_frame)
        self.btn_continue = ttk.Button(self.mainframe, text='continue', command=self.check_main_frame)
        self.btn_continue.grid(row=3, column=1, sticky=tk.W, padx=5)

    # frame for inputing
    def inputFrame(self):
        # get a number of vars and constraints
        nvars = int(self.get_nvars())
        ncons = int(self.get_nconstraints())

        # resize the window
        self.root.geometry(f'{300 + 50 * nvars + 50 * ncons}x{100 + 50 * ncons}')

        # raise messages
        if nvars == 0 or ncons == 0:
            # raise alarm
            return

        # children frame
        function_frame = tk.Frame(self.inputframe, background='white')
        function_frame.pack(fill='both')

        contraint_frame = tk.Frame(self.inputframe, background='white')
        contraint_frame.pack(fill='both')

        var_frame = tk.Frame(self.inputframe, background='white')
        var_frame.pack(fill='both')

        btn_frame = tk.Frame(self.inputframe, background='white')
        btn_frame.pack(fill='both')

        # function
        self.text_function = ttk.Label(function_frame, text='Function: ', justify='center', background='white',
                                       font=('Brass Mono', 10))
        self.text_function.grid(row=0, column=0, pady=5)

        k = 1
        for i in range(1, nvars * 2 + 1):
            if i % 2 != 0:
                self.text_field_function = ttk.Entry(function_frame, width=10)
                self.text_field_function.grid(row=0, column=i)
                self.c.append(self.text_field_function)
            else:
                if i == nvars * 2:
                    self.text_var_function = ttk.Label(function_frame, text=f'X{k}', background='white',
                                                       font=('Brass Mono', 10))
                else:
                    self.text_var_function = ttk.Label(function_frame, text=f'X{k}+', background='white',
                                                       font=('Brass Mono', 10))
                self.text_var_function.grid(row=0, column=i)
                k = k + 1

        # constraints
        self.text_constraints = ttk.Label(contraint_frame, text='constraints: ', background='white',
                                          font=('Brass Mono', 10))
        self.text_constraints.grid(row=0, column=0)

        for j in range(1, ncons + 1):
            k = 1
            for i in range(1, nvars * 2 + 1):
                if i % 2 != 0:
                    self.text_field_contraint = ttk.Entry(contraint_frame, width=10)
                    self.text_field_contraint.grid(row=j, column=i)
                    self.A.append(self.text_field_contraint)
                else:
                    if i == nvars * 2:
                        self.text_var_contraint = ttk.Label(contraint_frame, text=f'X{k} ', background='white',
                                                            font=('Brass Mono', 10))
                    else:
                        self.text_var_contraint = ttk.Label(contraint_frame, text=f'X{k}+', background='white',
                                                            font=('Brass Mono', 10))
                    self.text_var_contraint.grid(row=j, column=i)
                    k = k + 1

            eq_option = ['<=', '>=', '=']
            self.eq_field_constraint = ttk.Combobox(contraint_frame, values=eq_option, width=3, state='readonly')
            self.eq_field_constraint.grid(row=j, column=nvars * 2 + 1)
            self.eq_field_constraint.current(0)
            self.eq_constraint_list.append(self.eq_field_constraint)

            # self.eq_list.append(self.eq_field_constraint)
            self.text_field_b = ttk.Entry(contraint_frame, width=10)
            self.text_field_b.grid(row=j, column=nvars * 2 + 2)
            self.b.append(self.text_field_b)

        # variable constraint
        self.text_vc_name = ttk.Label(var_frame, text='constraint variables: ', background='white',
                                      font=('Brass Mono', 10))
        #self.text_vc_name.grid(row= 0, column=0, sticky='W')
        self.text_vc_name.pack(side='left',fill='x', pady=5)

        self.eq_constraint_var_list = []

        for i in range(1, nvars + 1):
            self.text_vc = ttk.Label(var_frame, text=f'X{i}', background='white', font=('Brass Mono', 10))
            #self.text_vc.grid(row=i, column=1, sticky='w')
            self.text_vc.pack(side='left',fill='x')

            eq_option_vc = ['<=0', '>=0', 'tu do']
            self.eq_field_vc = ttk.Combobox(var_frame, values=eq_option_vc, width=5, state='readonly')
            #self.eq_field_vc.grid(row=i, column=2, sticky='E', padx=5)
            self.eq_field_vc.pack(side='left',fill='x')
            self.eq_field_vc.current(1)

            self.eq_constraint_var_list.append(self.eq_field_vc)

        btn_frame.columnconfigure((0,1),weight=1)
        self.btn_solve = ttk.Button(btn_frame, text='Solve', command=self.check_input_frame, width=8)
        self.btn_solve.grid(row=0,column=0, pady=5, sticky='E')

        btn_back = ttk.Button(btn_frame, text='back', command=self.input_main_frame, width=8)
        btn_back.grid(row=0, column= 1, pady=5, sticky='W')

    def resultFrame(self, nvars, ncons, A, b , c, eq, eqv,sub):
        # resize the window
        #self.root.geometry(f'{200 + 50 * nvars}x{400 + 5*(10+nvars)}')
        self.root.geometry(f'{300 + 50 * nvars + 50 * ncons}x{300 + 50 * ncons}')
        #self.root.resizable(1,1)

        # raise messages
        if nvars == 0 or ncons == 0:
            # raise alarm
            return

        # children frame
        title_frame = tk.Frame(self.resultframe, background='white')
        title_frame.pack(fill='x')

        function_frame = tk.Frame(self.resultframe, background='white')
        function_frame.pack(fill='x')

        constraint_frame = tk.Frame(self.resultframe, background='white')
        constraint_frame.pack(fill='x')

        var_frame = tk.Frame(self.resultframe, background='white')
        var_frame.pack(fill='x')

        result_frame = tk.Frame(self.resultframe, background='white')
        result_frame.pack(fill='x')

        output_frame = tk.Frame(self.resultframe, background='white')
        output_frame.pack(fill='x')

        btn_frame = tk.Frame(self.resultframe, background='white')
        btn_frame.pack(fill='x')

        # the problem
        text_problem = ttk.Label(title_frame, text='The problem', justify='center', background='white',font=('Brass Mono', 15))
        #text_problem.grid(row=0, column=0, sticky='W', pady=5)
        text_problem.pack(side='top')

        # function
        text_function = ttk.Label(function_frame, text=f'Function ({sub}):', justify='center', background='white',font=('Brass Mono', 10))
        text_function.grid(row=0, column=0, pady=5)

        k = 0
        for i in range(1, (nvars * 2) + 1):
            if i % 2 != 0:
                text_field_function = ttk.Label(function_frame, text=f'{c[k]}',background='white' ,font=('Brass Mono', 10))
                text_field_function.grid(row=0, column=i)
            else:
                if i == nvars * 2:
                    text_var_function = ttk.Label(function_frame, text=f'X{k+1}', background='white',font=('Brass Mono', 10))

                else:
                    text_var_function = ttk.Label(function_frame, text=f'X{k + 1}+', background='white',font=('Brass Mono', 10))
                text_var_function.grid(row=0, column=i, pady=5)
                k = k + 1

        # constraints
        text_constraints = ttk.Label(constraint_frame, text='constraints: ', background='white',font=('Brass Mono', 10))
        text_constraints.grid(row=0, column=0)

        #print(A)
        g = 0
        for j in range(1, ncons + 1):
            k = 0
            for i in range(1, (nvars * 2) + 1):
                if i % 2 != 0:
                    text_contraint_value = ttk.Label(constraint_frame, text=f'{A[g]}', background='white',
                                                    font=('Brass Mono', 10))
                    text_contraint_value.grid(row=j, column=i)
                    g = g + 1
                else:
                    if i == nvars * 2:
                        text_contraint_var = ttk.Label(constraint_frame, text=f'X{k + 1}', background='white',
                                                      font=('Brass Mono', 10))
                    else:
                        text_contraint_var = ttk.Label(constraint_frame, text=f'X{k + 1}+', background='white',
                                                      font=('Brass Mono', 10))
                    text_contraint_var.grid(row=j, column=i, pady=5)
                    k = k + 1
            h = 0
            eq_field_constraint = ttk.Label(constraint_frame, text=f'{eq[j-1]}', background='white',font=('Brass Mono', 10))
            eq_field_constraint.grid(row=j, column=nvars * 2 + 1)

            text_field_b = ttk.Label(constraint_frame, text=f'{b[j-1]}', background='white',font=('Brass Mono', 10))
            text_field_b.grid(row=j, column=nvars * 2 + 2)
            h = h + 1

        # variable constraint
        text_vc_name = ttk.Label(var_frame, text='constraint sign: ', background='white',font=('Brass Mono', 10))
        text_vc_name.grid(row=0, column=0)

        for i in range(nvars):
            text_vc = ttk.Label(var_frame, text=f'X{i+1} {eqv[i]}', background='white', font=('Brass Mono', 10))
            text_vc.grid(row=i + 1, column=1)

        # result form simplex

        # transform eqv properly before applying function
        # eqv gui '>=0', '<=0', '=0'
        # eqv function 'x1>=0', .....
        original_list = eqv
        modified_list = [f'x{i + 1}' + s for i, s in enumerate(original_list)]

        #testing the output before applying simplex function
        """
        print(np.array(self.str2float(A)).reshape(ncons,nvars))
        print(np.array(self.str2float(c)))
        print(np.array(self.str2float(b)))
        print(modified_list)
        print(eq)
        print(sub)
        """

        result_title = ttk.Label(result_frame, text=f'result ', background='white', font=('Brass Mono', 15))
        result_title.pack(side='top')

        problem = simplex.Tableau(np.array(self.str2float(c)), np.array(self.str2float(A)).reshape(ncons,nvars) ,np.array(self.str2float(b)) ,modified_list ,eq, sub)
        # avoiding some inevitable errors in simplex class.
        try:
            lg = simplex.LinearProgram(problem)
            print(lg)

            #
            if lg.message == 'Linear programming unbounded':
                result_method = ttk.Label(output_frame, text=f'Method: {lg.main_method}', background='white',
                                          font=('Brass Mono', 10))
                result_method.grid(row=1, column=1, sticky='W')

                result_message = ttk.Label(output_frame, text=f'Status: {lg.message}', background='white',
                                           font=('Brass Mono', 10))
                result_message.grid(row=2, column=1, sticky='W')
                result_optimal_z = ttk.Label(output_frame, text=f'Optimized value: {lg.result_z}', background='white',
                                             font=('Brass Mono', 10))
                result_optimal_z.grid(row=3, column=1, sticky='W')

            #
            if lg.message == 'Linear programming infeasible':
                result_method = ttk.Label(output_frame, text=f'Method: {lg.main_method}', background='white',
                                          font=('Brass Mono', 10))
                result_method.grid(row=1, column=1, sticky='W')

                result_message = ttk.Label(output_frame, text=f'Status: {lg.message}', background='white',
                                           font=('Brass Mono', 10))
                result_message.grid(row=2, column=1, sticky='W')

            #
            if lg.message == 'Linear programming infinite solution' or lg.message == 'Linear programming solved':
                result_method = ttk.Label(output_frame, text=f'Method: {lg.main_method}', background='white', font=('Brass Mono', 10))
                result_method.grid(row = 1, column = 1, sticky='W')

                result_message = ttk.Label(output_frame, text=f'Status: {lg.message}', background='white', font=('Brass Mono', 10))
                result_message.grid(row = 2, column = 1, sticky='W')

                result_optimal_z = ttk.Label(output_frame, text=f'Optimized value: {lg.result_z}', background='white', font=('Brass Mono', 10))
                result_optimal_z.grid(row = 3, column = 1, sticky='W')

                for i in range(1,nvars+1):
                    result_x = ttk.Label(output_frame, text=f'X{i} = {lg.result_var[i-1]} ', background='white', font=('Brass Mono', 10))
                    result_x.grid(row = 4 + i, column = 1, sticky='W')
        except:
            error_line = ttk.Label(output_frame, text=f'Having errors in simplex function', background='white', font=('Brass Mono', 10))
            error_line.grid(row = 5, column = 1, sticky='W')

        # configure grid
        btn_frame.columnconfigure((0,1),weight=1)
        # button back

        btn_back = ttk.Button(btn_frame, text='back', command=self.result_main_frame, width=8)
        btn_back.grid(row=0,column=0, sticky='E')

        # button show graph
        btn_graph = ttk.Button(btn_frame, text='show figure',command=lambda: self.check_conditon_graph(problem,nvars), width=12)
        btn_graph.grid(row=0, column= 1,sticky='W')

    def check_conditon_graph(self, problem, nvars):
        print('chua toi if')
        if nvars == 2:
            print('toi nvars')
            lp = LP(problem.A_fig, problem.b_fig, problem.c_fig)
            print('toi lp')
            simplex_visual(lp).show()
            print('show')
        else:
            mess = tkinter.messagebox.showerror(title='Error', message='chi ho tro ve hai bien')

    # transitioning input frame to result frame
    def input_result_frame(self, nvars, ncons, matA, matb, matc, eq, eqv, subject):
        self.resultframe = tk.Frame(self.root, background='white')
        self.resultframe.pack(fill='both', expand='True')
        self.resultFrame(nvars, ncons, matA, matb , matc, eq, eqv, subject)

    # transitioning result frame to main frame
    def result_main_frame(self):
        self.root.destroy()
        App()

    # transitioning main frame to input frame
    def main_input_frame(self):
        #self.mainframe.forget()
        self.inputframe = tk.Frame(self.root, background='white')
        self.inputframe.pack(fill='both', expand='True')
        self.inputFrame()

    def input_main_frame(self):
        self.root.destroy()
        App()

if __name__ == '__main__':
    App()

