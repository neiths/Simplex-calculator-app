import numpy as np
import re


class UnboundedMessage(Exception):
    '''
    Muc dich cua class nay la de tao ra mot exception 
        khi ma nghiem cua bai toan nam trong vung khong gioi noi
    '''

    def __init__(self, message='Linear programming unbounded') -> None:
        self.message = message
        super().__init__(self.message)

    def __str__(self) -> str:
        return f'{self.message}'


class InfeasibleMeassage(Exception):
    '''
    Muc dich cua class nay la de tao ra mot exception
        khi ma bai toan khong co nghiem
    '''

    def __init__(self, message='Linear programming infeasible') -> None:
        self.message = message
        super().__init__(self.message)

    def __str__(self) -> str:
        return f'{self.message}'


class InfiniteSolution(Exception):
    '''
    Muc dich cua class nay la de tao ra mot exception
        khi ma bai toan co vo so nghiem
    '''

    def __init__(self, message='Linear programming infinite solution') -> None:
        self.message = message
        super().__init__(self.message)

    def check_inf(self, table):
        check = np.where(table[-1, 0:-1] == 0)[0]
        n_cons = table.shape[0] - 1
        for i in check:
            if len(np.where(table[:-1, i] != 0)[0]) == n_cons:
                return True
        return False

    def __str__(self) -> str:
        return f'{self.message}'


def idx_min(arr, idx):
    '''
    param: arr: array
    param: idx: index of array

    Method: 
        Muc dich tim ra index cua gia tri nho nhat 
            trong index cho truoc cua mang
    -----
    return: index of min value in array
    '''
    return np.where(arr[idx] == np.min(arr[idx]), idx, -1).max()

# Create a simplex method solve linear programing problem by dantzig rule and
# bland rule and two phrase method


class Tableau(object):
    '''
    Muc dich cua class nay la de tao ra mot tableu
        Voi A(nxm) la ma tran he so cua bai toan voi moi dong la moi rang buoc
            b(nx1) la vector he so tu do ung voi moi dong rang buoc
            c(1xm) la he so coefficient cua x trong ham muc tieu

        --------
        Result:
            Ket qua bai toan tra ve
            | x1 | x2 |... | xi | w1 | ... | wk | RHS |
            |a11| a12| ... |a1i| 1 | ... | 0    | b1 |
            | . | . |  ... | . | . | ... | .   | .  |
            |an1|an2| ... | . | . | ... | 1   | bn |
            |c1 | c2| ... |ci| 0  | ... | 0   | 0 |
    '''

    def __init__(self, c, A, b, condition_vars, condition_constraints, subject='min', on_figure = False) -> None:
        self.c = c
        self.A = A
        self.b = b
        self.condition_vars = condition_vars
        self.condition_constraints = condition_constraints
        
        self.subject = subject
        self.idx_vars = {i: i for i in range(len(c))}
        self.n_variable = len(self.idx_vars)
        self.on_figure = True if self.n_variable == 2 else False
        self.n_constraint = len(b)
        self.create_tableau()

    def __transform__(self):
        '''
        Method: 
            Noi ma tran A, vector b va c thanh mot bang tong quat
        '''
        self.AP = self.A
        self.bP = self.b
        self.cP = self.c

        
        # Kiem tra dieu kien cua bai toan (max or min)
        if self.subject == 'max':
            self.cP = -1*self.c

        # Transform condition variable
        n = self.n_variable
        c = 0
        for j in range(n):
            # Kiem tra dieu kien cua bien khi xi <= 0
            if self.condition_vars[j].lower().replace(' ', '') == f'x{j+1}<=0':
                self.AP[:, j] = -1*self.A[:, j]
                self.cP[j] = -1*self.c[j]
                c = 0

            # Kiem tra dieu kien cua bien khi xi la bien tu do
            elif self.condition_vars[j].lower() == f'x{j+1} free':
                # Kiem tra bien j+1 co phai la bien cua bai toan khong
                if j + 1 in self.idx_vars.keys():
                    self.idx_vars[j+1] += 1

                # Kiem tra co (voi co la the hien ta da gap mot bien tu do) va
                #   them cot moi vao bang canh bien vua xet
                idx = (j+2 if c == 1 else j+1)
                self.AP = np.insert(self.AP, idx, -1*self.A[:, j], axis=1)
                temp = (self.c[j] if self.subject == 'min' else -1*self.c[j])

                self.cP = np.insert(self.cP, idx, -1*temp, axis=0)
                self.n_variable += 1
                c = 1

        # Kiem tra dieu kien cua rang buoc dang thuc
        i = 0
        while i < self.n_constraint:
            k = 0

            match self.condition_constraints[i].lower().replace(' ', ''):
                # Kiem tra dieu kien cua rang buoc dang thuc khi >= 0
                #  nhan -1 cho ca hai ve ve.
                case '>=':
                    self.AP[i, :] = -1*self.AP[i, :]
                    self.bP[i] = -1*self.bP[i]

                # Kiem tra dieu kien cua rang buoc dang thuc khi = 0
                #  them mot rang buoc moi nhung nguoc dau cua rang buoc truoc do
                case '=':
                    self.AP = np.insert(self.AP, i+1, -1*self.AP[i, :], axis=0)
                    self.bP = np.insert(self.bP, i+1, -1*self.bP[i], axis=0)
                    self.condition_constraints = np.insert(
                        self.condition_constraints, i+1, '<=', axis=0)
                    self.n_constraint += 1
                    k = 1

            i = i + 1 + k

        if self.on_figure:
            self.A_fig = self.AP
            self.b_fig = self.bP.reshape(-1, 1)
            self.c_fig = self.cP*-1 if self.subject == 'max' else self.cP
            self.c_fig = self.c_fig.reshape(-1, 1)

    def create_tableau(self) -> None:
        '''
        Method:
            Trien khai bang tong quat tu bai toan dau A,b,c 
        '''

        # Create RHS column for tableu
        self.__transform__()
        self.rhs = np.array([0] * (self.n_constraint + 1))
        np.put(self.rhs, np.arange(0, self.n_constraint), self.bP)

        # Create tableu by combine A and RHS
        temp = self.AP
        temp = np.concatenate(
            (temp, np.eye(self.n_constraint, self.n_constraint)), axis=1)

        c_temp = np.array([0] * (self.n_constraint + self.n_variable))

        np.put(c_temp, np.arange(0, self.n_variable), self.cP)
        temp = np.concatenate((temp, c_temp.reshape(1, -1)), axis=0)
        self.tableu = np.concatenate((temp, self.rhs.reshape(-1, 1)), axis=1)

    def __str__(self) -> str:
        '''
        Method:
            In ra cac thong tin luu giu tu bai toan dau vao
        '''
        return f'Subject: {self.subject} \nCondition: {self.condition_vars} \nC: {self.c} \nA: {self.A} \nb: {self.b}'


# Solve linear programming by Dantzig rule
class Dantzig(object):
    '''
    Method:
        Trien khai giai thuat Dantzig de giai bai toan toi uu tuyen tinh
            bang cach:
                Voi bien vao tim he so am va nho nhat trong ham muc tieu
                Voi bien ra tu vi tri cot bien vao tim hang pivot dua vao ti le bi/ai la so duong nho nhat
        Truong hop khi nhan thay bi = 0 hoac bai toan lap nhu bai toan dau
            thuat toan Dantzig khi nay se chuyen sang giai thuat Bland
                Voi bien vao ta tim chi so nho nhat trong cac he so am
                Voi bien ra tuong tu nhu Dantzig
    '''

    def __init__(self, table, idx_vars) -> None:
        self.table = table.copy()
        self.idx_vars = idx_vars
        self.mem_table = self.table.copy()

    def find_pivot(self, col, on_bland=False):
        '''
        param: col: int - chi so cot bien vao
        param: on_bland: bool - True: su dung giai thuat Bland

        Method: 
            Tim chi so hang pivot dua vao ti le bi/ai la so duong nho nhat
        --------------------
        Return: int - chi so hang pivot
        '''

        # Tinh ti le bi/ai tuong ung moi hang
        #   Neu ai > 0 ta gan ti le bi/ai
        #   Neu ai <= 0 ta gan ti le ai
        temp = np.array([self.table[i, -1] / self.table[i, col]
                        if (self.table[i, col] > 0)
                        else self.table[i, col]
                        for i in range(self.table.shape[0]-1)])
        # Neu thuat toan su dung Bland ta chi lay cac ti le bi/ai >= 0
        if on_bland == True:
            mask = np.ma.masked_less(temp, 0)

        # Nguoc lai ta lay cac ti le bi/ai > 0
        else:
            mask = np.ma.masked_less_equal(temp, 0)

        # Viec chi tim 1 dong co ti le bi/ai nho nhat chua bao quat van de
        #   nen ta can tim tat ca cac dong co ti le bi/ai nho nhat
        #   va chon ra he so ai nho nhat la pivot duoc xem xet
        idx = np.where(mask == mask.min())
        return idx[0] if len(idx) != len(temp) and mask.min() != '--' else None

    def solve(self,
              n_ter_limit=None,
              on_bland=False):
        '''
        param: n_ter_limit: int - gioi han so vong lap
        param: on_bland: bool - True: su dung giai thuat Bland

        Method: Tim tu vung toi uu bang giai thuat Dantzig
            Dantzig: so dong duoc chon la he so am nho nhat
            Bland: so dong duoc chon la chi so nho nhat ma co he so am
        -------
        Return:
            table: np.array - bang tu vung toi uu
            message: str - thong bao ket qua
        '''
        iter = 0
        message = None

        # Xet lan luot trang thai cua ham muc tieu
        while np.where(self.table[-1, 0:-1] < 0):
            col = 0

            # Neu su dung giai thuat Bland hoac
            #   tu vung lap lai sau n buoc so voi tu vung ban dau,
            #   khi do xet vi tri cot bien vao co chi so nho nhat
            if on_bland == True or \
                    ((self.mem_table == self.table).all() and
                        iter > 0):
                cond_col = np.where(np.ma.masked_greater_equal(
                    self.table[-1, 0:-1], 0) < 0)
                col = np.min(cond_col[0]) if len(cond_col[0]) > 0 else None

            # Nguoc lai neu la Dantzig ta chi xet bien vao cho he so am nho nhat
            #       trong ham muc tieu
            elif (on_bland == False):
                col = np.argmin(self.table[-1, 0:-1])

            # Kiem tra cot vua tim co thoa 2 tieu chi tren
            check = self.table[-1, col] if col != None else 0

            # Neu gia tri he so am o ham muc tieu thi thuc hien phep xoay tren tu vung
            if check < 0:

                # Tim dong thoa 2 tieu chi thuat toan
                row = self.find_pivot(col, on_bland)

                # Neu ta tim duoc 1 hoac nhieu hon 2 dong co cung tieu chi
                if isinstance(row, np.ndarray):

                    # Tim ra dong co he so ai nho nhat de lam pivot
                    row = idx_min(self.table[:-1, col].reshape(-1), row)

                    # Chia he so tai dong cho he so pivot
                    self.table[row, :] = self.table[row, :] / \
                        self.table[row, col]

                    # Thuc hien phep xoay nhu phep Guass-Jordan tren ma tran
                    for i in range(self.table.shape[0]):
                        if i != row:
                            self.table[i, :] = self.table[i, :] - \
                                self.table[row, :] * self.table[i, col]

                # Neu ta khong tim duoc dong thoa 2 tieu chi
                #   tuc co bien vao ma khong bien ra
                #   -> khang dinh bai toan khong gioi noi
                else:
                    message = str(UnboundedMessage())
                    break

            # Neu he so ham muc tieu toan duong
            #  Kiem tra neu he so ci = 0 nhung cac rang buoc != 0 -> Bai toan vo so nghiem
            #  Kiem tra neu he so ci != 0 va cac rang buoc != 0 -> Bai toan co nghiem
            #  Dong thoi thoat ra va ket thuc thuat toan
            else:
                show = InfiniteSolution()
                if show.check_inf(self.table):
                    message = str(show)
                else:
                    message = 'Linear programming solved'
                break
            iter += 1
        return self.table, message


class Two_Phase(object):
    '''
    Method: Su dung thuat toan don hinh 2 pha giai bai toan toi uu tuyen tinh
                trong truong hop xuat hien b < 0
    '''

    def __init__(self, table, idx_vars):
        self.table = table.copy()
        self.idx_vars = idx_vars
        self.z = self.table[-1, :].copy()

        # Sau khi co thong tin bai toan dau
        #   thuc hien buoc lap tu vung cho pha 1
        self.phase1_tableau()
        self.n_phase1 = self.table.shape[1]

    def phase1_tableau(self):
        '''
        Method: Thuc hien buoc lap tu vung cho pha 1
            them mot bien gia x0  vao bang va thiet lap dieu kien can thiet
            de tim nghiem toi uu cho pha 1
        '''
        n_vars = self.table.shape[1]
        n_cons = self.table.shape[0] - 1
        self.table = np.insert(self.table, n_vars-1, -
                               1*np.ones(n_cons + 1), axis=1)
        self.table[-1, :] = 0
        self.table[-1, -2] = 1

    def phase2_tableau(self):
        '''
        Method: Thuc hien buoc lap tu vung cho pha 2 
                tu dieu kien rang buoc o pha 1, loai bo bien gia x0
                cap nhat lai bien o ham muc tieu cho tuong xung voi 
                cac rang buoc
        '''

        # Xoa cot x0
        self.table = np.delete(self.table, np.s_[-2], axis=1)

        # Cap nhat lai he so ham muc tieu
        zz = np.array([0] * (self.table.shape[1]), dtype=np.float64)
        for i in self.idx_vars:
            row = np.where(self.table[:, i] == 1)
            if len(row[0]) == 1:
                temp = self.table[row, :].reshape(-1)
                temp[i] = 0
                zz += self.z[i]*temp*-1
            else:
                zz[i] += self.z[i]
        self.table[-1, :] = zz

    def find_pivot(self, col):
        '''
        param: col: int - chi so cot bien vao

        Method: Ham tim chi so  hang pivot dua vao 
                    ti le bi/ai la  nho nhat cho viec tim pivot o pha 1
        -----
        Return: int - chi so hang pivot
        '''
        temp = np.array([self.table[i, -1] / self.table[i, col]
                         if (self.table[i, col] > 0)
                         else self.table[i, col]
                        for i in range(self.table.shape[0]-1)])
        mask = np.ma.masked_less_equal(temp, 0)
        return mask.argmin() if str(mask.min()) != '--' else None

    def dantzig_2Phase(self, on_bland=False):
        '''
        param: on_bland: bool - thiet lap su dung phuong phap bland

        Method: Thuc hien giai bai toan toi uu tuyen tinh 
                    bang phuong phap Dantzig nhung 
                    dieu kien ket thuc tu vung khac 
                    khi kiem tra gia tri toi uu cua ham muc tieu pha 1 voi viec cho x0 = 0
        ------
        Return: 
            table: np.array - bang giai bai toan toi uu
            message: str - thong bao ket qua

        '''
        message = None
        while True:
            if on_bland == True:
                cond_col = np.where(np.ma.masked_greater_equal(
                    self.table[-1, 0:-1], 0) < 0)
                col = np.min(cond_col[0]) if len(cond_col[0]) > 0 else None

            elif on_bland == False:
                col = np.argmin(self.table[-1, 0:-1])

            check = self.table[-1, col] if col != None else 0
            print(self.table)
            if check < 0:
                row = self.find_pivot(col)

                # Neu ta khong tim duoc dong thoa 2 tieu chi
                #   tuc co bien vao ma khong bien ra
                #   -> khang dinh bai toan khong gioi noi
                if row == None:
                    message = str(UnboundedMessage())

                else:
                    # Chia he so tai dong cho he so pivot
                    self.table[row, :] = self.table[row, :] / \
                        self.table[row, col]

                    # Thuc hien phep xoay nhu phep Guass-Jordan tren ma tran
                    for i in range(self.table.shape[0]):
                        if i != row:
                            self.table[i, :] = self.table[i, :] - \
                                self.table[row, :] * self.table[i, col]

            #  Neu he so x0 = 1 va gia tri toi uu la 0 khi cho x0 = 0 -> Chuyen sang pha 2
            #  Nguoc lai ham muc tieu toi uu nhung ton tai cac bien khong co so va gia tri toi uu != 0 -> Bai toan vo nghiem
            #  Dong thoi thoat ra va ket thuc thuat toan
            else:
                if self.n_phase1 == self.table.shape[1]:
                    if sum(self.table[-1, :]) == 1 and self.table[-1, -1] == 0:
                        message = 'Linear programming solved'
                    else:
                        message = str(InfeasibleMeassage())
                    break
        return self.table, message

    def solve(self,
              on_bland=False):
        '''
        param: on_bland: bool - thiet lap su dung phuong phap bland

        Method: Thuc hien giai bai toan toi uu tuyen tinh theo tung pha
            Pha 1: tim nghiem toi uu khi x0 = 0 va cac he so b >= 0
            Pha 2: Tu cac rang buoc pha 1 da duoc xoay, tim nghiem toi uu voi Dantzig
        -----
        Return: 
            table: np.array - bang giai bai toan toi uu
            message: str - thong bao ket qua
        '''
        message = None
        # Tao tu vung cho pha I
        col = np.where(self.table[-1, 0:-1] > 0)[0]
        row = np.ma.masked_greater_equal(self.table[0:-1, -1], 0).argmin()
        self.table[row, :] = self.table[row, :] / self.table[row, col]
        for i in range(self.table.shape[0]):
            if i != row:
                self.table[i, :] = self.table[i, :] - \
                    self.table[row, :] * self.table[i, col]

        # Don Hinh Dantzig cho pha 1
        self.table, message = self.dantzig_2Phase(on_bland=on_bland)
        # Kiem tra thong bao tu Pha I, neu thanh cong tao tu vung pha II
        if message != str(InfeasibleMeassage()):
            self.phase2_tableau()
            if len(np.where((self.table[0:-1, -1]) == 0)[0]) > 0:
                on_bland = True
            solve_prb = Dantzig(self.table, self.idx_vars)
            self.table, message = solve_prb.solve(on_bland=on_bland)
        return self.table, message


class LinearProgram(object):
    ''' 
    Method: 
        Muc dich cua class nay de luu tru thong tin bang tu vung va
            thuc hien cac thao tac tren tu vung de tim ham muc tieu 
            thoa cac rang buoc
    '''

    def __init__(self, tableau):
        self.ori_table = tableau.tableu
        self.table = tableau.tableu.copy()
        self.idx_vars = tableau.idx_vars
        self.n_vars = len(self.idx_vars)
        self.obj = tableau.subject
        self.n_constraints = tableau.n_constraint
        self.condition_vars = tableau.condition_vars

        self.main_method = None
        self.result_var = np.zeros(self.n_vars)
        self.result_z = 0
        self.message = None

    def __find__(self):
        '''
        Method: Phuong thuc nay tim ra gia tri cua bien va ham muc tieu 
                    thong qua tu vung cuoi cung cua bai toan
        '''
        flag = 1 if self.obj == 'max' else -1
        self.result_z = round(self.table[-1, -1]*flag, 2)
        match self.message:

            # Neu bai toan khong gioi noi tra ve gia tri infinite tuong ung
            case 'Linear programming unbounded':
                self.result_z = np.inf*flag

            case 'Linear programming infeasible':
                self.result_z = np.nan
            # TH bai toan co nghiem
            case 'Linear programming solved':

                # Duyet qua tung vi tri cua bien trong tu vung
                for idx in self.idx_vars:
                    # Neu vi tri bien do tai cot la bien khong co so thi gan 0
                    if len(np.where(self.table[:-1, self.idx_vars[idx]] != 0)[0]) > 1:
                        self.result_var[idx] = 0

                    # Nguoc lai neu bien la bien co so thi gan gia tri tuong ung
                    else:
                        temp = np.where(
                            self.table[:-1, self.idx_vars[idx]] == 1)[0]
                        if not len(temp):
                            continue
                        row = temp[0]
                        self.result_var[idx] = round(self.table[row, -1], 2)
                        if re.findall(r'<=', self.condition_vars[idx]):
                            self.result_var[idx] *= -1

    def __solve__(self):
        '''
        Method: Phuong thuc nay thuc hien giai bai toan theo cac dieu kien sau:
            1. Neu xuat hien b am: Phuong phap 2 pha
            2. Neu xuat hien b = 0: Phuong phap Bland
            3. Nguoc lai: Phuong phap Dantzig

        -----
        Return: 
            1. Gia tri cua bien va ham muc tieu
            2. Thong bao ve tinh trang bai toan
        '''

        # Kiem tra su ton tai cua b < 0
        if len(np.where((self.table[0:-1, -1]) < 0)[0]) > 0:
            self.main_method = 'Two-Phrase'
            tp = Two_Phase(self.table, self.idx_vars)
            self.table, self.message = tp.solve()

        # Kiem tra su ton tai cua b = 0
        elif len(np.where(self.table[0:-1, -1] == 0)[0]) > 0:
            self.main_method = 'Bland'
            dantzig = Dantzig(self.table, self.idx_vars)
            self.table, self.message = dantzig.solve(on_bland=True)

        # Truong hop con lai khi b > 0
        else:
            self.main_method = 'Dantzig'
            dantzig = Dantzig(self.table, self.idx_vars)
            self.table, self.message = dantzig.solve()

        # Tim gia tri cua bien va ham muc tieu tu cac thuat toan tren
        self.__find__()

    def __str__(self) -> str:
        '''
        Method: Phuong thuc nay tra ve thong tin cua bai toan bang dang chuoi 
            1. Bang tu vung
            2. Thong bao ve tinh trang bai toan
            3. Gia tri cua bien va ham muc tieu
        '''
        self.__solve__()
        return f'Primary problem: \n {self.ori_table} ' + \
            '   \nSolution: ' +\
            f'\nOptimize value: z = {self.result_z} \n' + \
            str('\n'.join(
                [f'x{i+1} = {self.result_var[i]}' for i in range(self.n_vars)]))
