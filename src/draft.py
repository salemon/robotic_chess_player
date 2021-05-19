from io import BytesIO as StringIO
import numpy as np

def board2fen(board):
        board = np.rot90(board,2)
        with StringIO() as s:
            for row in board:
                empty = 0
                for cell in row:
                    if cell != '_':
                        if empty > 0:
                            s.write(str(empty))
                            empty = 0
                        s.write(cell)
                    else:
                        empty += 1
                if empty > 0:
                    s.write(str(empty))
                s.write('/')
            # Move one position back to overwrite last '/'
            s.seek(s.tell() - 1)
            # If you do not have the additional information choose what to put
            s.write(' b KQkq - 0 1')
            print(s.getvalue())
            return None

            
board = np.array([['K', 'B', 'R', 'B', 'R', 'N', 'N', 'k'],
       ['_', '_', '_', '_', '_', '_', '_', '_'],
       ['_', '_', '_', '_', '_', '_', '_', '_'],
       ['_', '_', '_', '_', '_', '_', '_', '_'],
       ['_', '_', '_', '_', '_', '_', '_', '_'],
       ['p', '_', '_', '_', '_', '_', '_', '_'],
       ['p', '_', '_', '_', '_', '_', '_', '_'],
       ['q', 'r', 'r', 'b', 'n', 'b', 'n', 'Q']])

board2fen(board)