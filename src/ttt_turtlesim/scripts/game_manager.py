#!/usr/bin/env python3

"""
Tic-Tac-Toe Game Manager Node
Manages game state and provides move validation service
"""

import rospy
from ttt_turtlesim.srv import ValidateMove, ValidateMoveResponse, GetGameState, GetGameStateResponse
from std_msgs.msg import String

class GameManager:
    def __init__(self):
        rospy.init_node('game_manager')
        
        # Initialize 3x3 game board
        self.board = [['', '', ''], ['', '', ''], ['', '', '']]
        self.game_over = False
        self.winner = None
        self.current_player = 'X'  # X always starts
        
        # Create services
        self.validate_service = rospy.Service('validate_move', ValidateMove, self.handle_validate_move)
        self.get_state_service = rospy.Service('get_game_state', GetGameState, self.handle_get_state)
        
        # Publisher for game state
        self.state_pub = rospy.Publisher('game_state', String, queue_size=10)
        
        rospy.loginfo("Tic-Tac-Toe Game Manager is ready!")
        self.print_board()
        
    def handle_validate_move(self, req):
        """Handle move validation request and make the move if valid"""
        response = ValidateMoveResponse()
        
        # Check if move is within bounds
        if req.row < 0 or req.row > 2 or req.col < 0 or req.col > 2:
            response.is_valid = False
            response.error_message = "Row and column must be between 0 and 2"
            response.board_state = self.get_board_state_array()
            return response
        
        # Check if cell is already occupied
        if self.board[req.row][req.col] != '':
            response.is_valid = False
            response.error_message = "Cell is already occupied"
            response.board_state = self.get_board_state_array()
            return response
        
        # Check if game is already over
        if self.game_over:
            response.is_valid = False
            response.error_message = "Game is already over"
            response.board_state = self.get_board_state_array()
            return response
        
        # Check if it's the correct player's turn
        if req.player_symbol != self.current_player:
            response.is_valid = False
            response.error_message = f"It's {self.current_player}'s turn, not {req.player_symbol}'s turn"
            response.board_state = self.get_board_state_array()
            return response
        
        # Check if player symbol is valid
        if req.player_symbol not in ['X', 'O']:
            response.is_valid = False
            response.error_message = "Player symbol must be 'X' or 'O'"
            response.board_state = self.get_board_state_array()
            return response
        
        # If we get here, the move is valid - make the move
        if self.make_move(req.row, req.col, req.player_symbol):
            response.is_valid = True
            response.error_message = ""
            response.board_state = self.get_board_state_array()
        else:
            response.is_valid = False
            response.error_message = "Failed to make move"
            response.board_state = self.get_board_state_array()
        
        return response
    
    def handle_get_state(self, req):
        """Handle request to get current game state"""
        response = GetGameStateResponse()
        response.board_state = self.get_board_state_array()
        response.game_over = self.game_over
        response.winner = self.winner if self.winner else ""
        response.current_player = self.current_player if not self.game_over else ""
        response.error_message = ""
        return response
    
    def make_move(self, row, col, symbol):
        """Actually make a move on the board"""
        if 0 <= row <= 2 and 0 <= col <= 2 and self.board[row][col] == '' and not self.game_over:
            self.board[row][col] = symbol
            self.check_game_status()
            
            # Switch to the other player if game is not over
            if not self.game_over:
                self.current_player = 'O' if self.current_player == 'X' else 'X'
                
            self.print_board()
            return True
        return False
    
    def check_game_status(self):
        """Check if game is won or drawn"""
        # Check rows
        for row in self.board:
            if row[0] == row[1] == row[2] and row[0] != '':
                self.game_over = True
                self.winner = row[0]
                rospy.loginfo(f"Game Over! Player {self.winner} wins!")
                return
        
        # Check columns
        for col in range(3):
            if self.board[0][col] == self.board[1][col] == self.board[2][col] and self.board[0][col] != '':
                self.game_over = True
                self.winner = self.board[0][col]
                rospy.loginfo(f"Game Over! Player {self.winner} wins!")
                return
        
        # Check diagonals
        if self.board[0][0] == self.board[1][1] == self.board[2][2] and self.board[0][0] != '':
            self.game_over = True
            self.winner = self.board[0][0]
            rospy.loginfo(f"Game Over! Player {self.winner} wins!")
            return
        
        if self.board[0][2] == self.board[1][1] == self.board[2][0] and self.board[0][2] != '':
            self.game_over = True
            self.winner = self.board[0][2]
            rospy.loginfo(f"Game Over! Player {self.winner} wins!")
            return
        
        # Check for draw
        is_board_full = True
        for row in self.board:
            for cell in row:
                if cell == '':
                    is_board_full = False
                    break
            if not is_board_full:
                break
        
        if is_board_full:
            self.game_over = True
            self.winner = None
            rospy.loginfo("Game Over! It's a draw!")
    
    def get_board_state_array(self):
        """Convert 2D board to 1D array for service response"""
        state = []
        for row in self.board:
            for cell in row:
                state.append(cell if cell != '' else ' ')
        return state
    
    def print_board(self):
        """Print the current board state to console"""
        rospy.loginfo("Current board state:")
        for i, row in enumerate(self.board):
            row_str = " | ".join([cell if cell != '' else ' ' for cell in row])
            rospy.loginfo(f" {row_str} ")
            if i < 2:
                rospy.loginfo("-----------")
        if not self.game_over:
            rospy.loginfo(f"Current player: {self.current_player}")
        else:
            if self.winner:
                rospy.loginfo(f"Winner: {self.winner}")
            else:
                rospy.loginfo("It's a draw!")

if __name__ == '__main__':
    try:
        gm = GameManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass