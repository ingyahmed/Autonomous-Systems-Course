#!/usr/bin/env python3

"""
Tic-Tac-Toe Game Client
Terminal interface for playing Tic-Tac-Toe with turtlesim
"""

import rospy
import actionlib
from ttt_turtlesim.srv import ValidateMove, GetGameState
from ttt_turtlesim.msg import ExecuteMoveGoal, ExecuteMoveAction
import sys

def print_board_visual(board_state):
    """Print a visual representation of the board"""
    print("\nCurrent board:")
    print(f" {board_state[0]} | {board_state[1]} | {board_state[2]} ")
    print("-----------")
    print(f" {board_state[3]} | {board_state[4]} | {board_state[5]} ")
    print("-----------")
    print(f" {board_state[6]} | {board_state[7]} | {board_state[8]} ")
    print()

def get_player_move(current_player):
    """Get move from the current player via terminal input"""
    print(f"Player {current_player}'s turn")
    try:
        row = int(input("Enter row (0-2): "))
        col = int(input("Enter column (0-2): "))
        
        if row < 0 or row > 2 or col < 0 or col > 2:
            print("Invalid input! Row and column must be between 0 and 2.")
            return get_player_move(current_player)
        
        return row, col
    except ValueError:
        print("Invalid input! Please enter numbers only.")
        return get_player_move(current_player)

def main():
    rospy.init_node('game_client')
    
    # Wait for services and actions
    rospy.wait_for_service('validate_move')
    rospy.wait_for_service('get_game_state')
    validate_client = rospy.ServiceProxy('validate_move', ValidateMove)
    get_state_client = rospy.ServiceProxy('get_game_state', GetGameState)
    
    move_client = actionlib.SimpleActionClient('execute_move', ExecuteMoveAction)
    move_client.wait_for_server()
    
    print("Welcome to Tic-Tac-Toe with Turtlesim!")
    print("Players will take turns entering row and column positions.")
    print("Player X goes first, followed by Player O.")
    
    # Initialize with empty board
    initial_board = [' '] * 9
    print_board_visual(initial_board)
    
    game_over = False
    
    while not game_over and not rospy.is_shutdown():
        try:
            # Get the current game state
            state_response = get_state_client()
            
            # Check if game is already over
            if state_response.game_over:
                print_board_visual(state_response.board_state)
                if state_response.winner:
                    print(f"Game Over! Player {state_response.winner} wins!")
                else:
                    print("Game Over! It's a draw!")
                game_over = True
                break
            
            current_player = state_response.current_player
            
            # Get move from current player
            row, col = get_player_move(current_player)
            
            # Validate and make the move
            response = validate_client(row, col, current_player)
            
            if not response.is_valid:
                print(f"Invalid move: {response.error_message}")
                continue
            
            # Execute the move via action server
            goal = ExecuteMoveGoal()
            goal.row = row
            goal.col = col
            goal.player_symbol = current_player
            
            # Send goal to action server
            move_client.send_goal(goal)
            
            # Wait for result
            move_client.wait_for_result()
            result = move_client.get_result()
            
            if result.success:
                print(f"Move {current_player} at ({row}, {col}) executed successfully!")
                
                # Get updated board after the move
                updated_state = get_state_client()
                print_board_visual(updated_state.board_state)
                
                # Check if game is now over
                if updated_state.game_over:
                    game_over = True
                    if updated_state.winner:
                        print(f"Game Over! Player {updated_state.winner} wins!")
                    else:
                        print("Game Over! It's a draw!")
                
            else:
                print(f"Failed to execute move: {result.error_message}")
                
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    
    print("Game finished!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nGame interrupted by user.")
    except Exception as e:
        print(f"An error occurred: {e}")