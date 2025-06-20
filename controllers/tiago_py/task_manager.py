"""
Task Queue Management System

This module extends KeyboardReader functionality to handle multiple sequential
navigation tasks with proper queuing and priority management.

Author: Mission 2 Navigation System
"""

import time
from typing import List, Optional, Dict, Tuple
from enum import Enum
from collections import deque

from keyboardreader import KeyboardReader


class TaskPriority(Enum):
    """Priority levels for navigation tasks."""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    EMERGENCY = 4


class TaskStatus(Enum):
    """Status of navigation tasks."""
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class NavigationTask:
    """
    Represents a single navigation task with metadata.
    """
    
    def __init__(self, target: str, priority: TaskPriority = TaskPriority.NORMAL,
                 task_id: Optional[str] = None):
        """
        Initialize a navigation task.
        
        Args:
            target: Target location (red, green, ducks, balls)
            priority: Task priority level
            task_id: Optional unique task identifier
        """
        self.target = target
        self.priority = priority
        self.task_id = task_id or f"task_{int(time.time() * 1000)}"
        self.status = TaskStatus.PENDING
        self.created_time = time.time()
        self.start_time = None
        self.completion_time = None
        self.attempts = 0
        self.max_attempts = 3
        
    def start(self):
        """Mark task as started."""
        self.status = TaskStatus.ACTIVE
        self.start_time = time.time()
        self.attempts += 1
        
    def complete(self):
        """Mark task as completed."""
        self.status = TaskStatus.COMPLETED
        self.completion_time = time.time()
        
    def fail(self):
        """Mark task as failed."""
        self.status = TaskStatus.FAILED
        self.completion_time = time.time()
        
    def cancel(self):
        """Mark task as cancelled."""
        self.status = TaskStatus.CANCELLED
        self.completion_time = time.time()
        
    def can_retry(self) -> bool:
        """Check if task can be retried."""
        return self.attempts < self.max_attempts and self.status == TaskStatus.FAILED
        
    def get_duration(self) -> Optional[float]:
        """Get task execution duration."""
        if self.start_time and self.completion_time:
            return self.completion_time - self.start_time
        return None
        
    def __str__(self) -> str:
        return f"Task({self.task_id}: {self.target}, {self.status.value}, priority={self.priority.value})"


class TaskManager:
    """
    Manages navigation task queue with priority handling and keyboard input.
    
    Extends KeyboardReader functionality to provide robust task management
    with queuing, prioritization, and retry mechanisms.
    """
    
    def __init__(self, keyboard_reader: KeyboardReader):
        """
        Initialize the task manager.
        
        Args:
            keyboard_reader: KeyboardReader instance for input handling
        """
        self.keyboard_reader = keyboard_reader
        self.task_queue = deque()
        self.current_task = None
        self.completed_tasks = []
        self.failed_tasks = []
        
        # Task management parameters
        self.max_queue_size = 10
        self.task_timeout = 120.0  # 2 minutes per task
        self.retry_delay = 5.0  # Delay before retry
        
        # Valid targets
        self.valid_targets = {'red', 'green', 'ducks', 'balls'}
        
        # Statistics
        self.stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'cancelled_tasks': 0
        }
        
    def process_keyboard_input(self) -> Optional[str]:
        """
        Process keyboard input and convert to navigation tasks.

        Returns:
            Target name if new task should be started, None otherwise
        """
        # Get keyboard input using the existing get_command method
        command = self.keyboard_reader.get_command()

        if command is None:
            return self.get_next_task_target()

        # Process command string (may contain multiple targets separated by commas)
        targets = [target.strip() for target in command.split(',')]

        for target in targets:
            if target in self.valid_targets:
                # Add task to queue with normal priority
                self.add_task(target, TaskPriority.NORMAL)
                print(f"Added task: {target}")
            elif target == 'cancel' or target == 'c':
                self.cancel_current_task()
                print("Current task cancelled")
            elif target == 'clear' or target == 'q':
                self.clear_queue()
                print("Task queue cleared")
            elif target == 'status' or target == 's':
                self.print_status()
            else:
                print(f"Unknown command: {target}")

        return self.get_next_task_target()
    
    def add_task(self, target: str, priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """
        Add a new navigation task to the queue.
        
        Args:
            target: Target location name
            priority: Task priority level
            
        Returns:
            True if task was added successfully, False otherwise
        """
        if target not in self.valid_targets:
            print(f"Invalid target: {target}")
            return False
            
        if len(self.task_queue) >= self.max_queue_size:
            print("Task queue is full")
            return False
            
        # Create new task
        task = NavigationTask(target, priority)
        
        # Insert task based on priority
        inserted = False
        for i, existing_task in enumerate(self.task_queue):
            if task.priority.value > existing_task.priority.value:
                self.task_queue.insert(i, task)
                inserted = True
                break
                
        if not inserted:
            self.task_queue.append(task)
            
        self.stats['total_tasks'] += 1
        return True
    
    def get_next_task_target(self) -> Optional[str]:
        """
        Get the target for the next task to execute.
        
        Returns:
            Target name if there's a pending task, None otherwise
        """
        # Check if current task is still active
        if self.current_task and self.current_task.status == TaskStatus.ACTIVE:
            # Check for timeout
            if (self.current_task.start_time and 
                time.time() - self.current_task.start_time > self.task_timeout):
                print(f"Task {self.current_task.task_id} timed out")
                self.current_task.fail()
                self._handle_task_completion()
            else:
                return None  # Current task still running
        
        # Get next task from queue
        if self.task_queue:
            self.current_task = self.task_queue.popleft()
            self.current_task.start()
            print(f"Starting task: {self.current_task}")
            return self.current_task.target
            
        return None
    
    def complete_current_task(self):
        """Mark the current task as completed."""
        if self.current_task and self.current_task.status == TaskStatus.ACTIVE:
            self.current_task.complete()
            print(f"Task completed: {self.current_task.task_id}")
            self._handle_task_completion()
    
    def fail_current_task(self):
        """Mark the current task as failed."""
        if self.current_task and self.current_task.status == TaskStatus.ACTIVE:
            self.current_task.fail()
            print(f"Task failed: {self.current_task.task_id}")
            self._handle_task_completion()
    
    def cancel_current_task(self):
        """Cancel the current task."""
        if self.current_task and self.current_task.status == TaskStatus.ACTIVE:
            self.current_task.cancel()
            print(f"Task cancelled: {self.current_task.task_id}")
            self._handle_task_completion()
    
    def _handle_task_completion(self):
        """Handle completion of current task."""
        if not self.current_task:
            return
            
        # Update statistics
        if self.current_task.status == TaskStatus.COMPLETED:
            self.completed_tasks.append(self.current_task)
            self.stats['completed_tasks'] += 1
            
        elif self.current_task.status == TaskStatus.FAILED:
            # Check if task can be retried
            if self.current_task.can_retry():
                print(f"Retrying task: {self.current_task.target} (attempt {self.current_task.attempts + 1})")
                # Reset the current task for retry instead of creating new one
                self.current_task.status = TaskStatus.PENDING
                self.task_queue.appendleft(self.current_task)
            else:
                self.failed_tasks.append(self.current_task)
                self.stats['failed_tasks'] += 1
                
        elif self.current_task.status == TaskStatus.CANCELLED:
            self.stats['cancelled_tasks'] += 1
            
        self.current_task = None
    
    def clear_queue(self):
        """Clear all pending tasks from the queue."""
        cancelled_count = len(self.task_queue)
        self.task_queue.clear()
        self.stats['cancelled_tasks'] += cancelled_count
        print(f"Cleared {cancelled_count} pending tasks")
    
    def get_queue_status(self) -> Dict:
        """
        Get current queue status information.
        
        Returns:
            Dictionary with queue status details
        """
        return {
            'current_task': str(self.current_task) if self.current_task else None,
            'queue_length': len(self.task_queue),
            'pending_tasks': [str(task) for task in self.task_queue],
            'statistics': self.stats.copy()
        }
    
    def print_status(self):
        """Print current task manager status."""
        print("\n=== Task Manager Status ===")
        print(f"Current Task: {self.current_task or 'None'}")
        print(f"Queue Length: {len(self.task_queue)}")
        
        if self.task_queue:
            print("Pending Tasks:")
            for i, task in enumerate(self.task_queue):
                print(f"  {i+1}. {task}")
                
        print(f"Statistics: {self.stats}")
        print("===========================\n")
    
    def has_pending_tasks(self) -> bool:
        """Check if there are pending tasks."""
        return len(self.task_queue) > 0 or (
            self.current_task and self.current_task.status == TaskStatus.ACTIVE
        )
    
    def get_task_history(self) -> List[NavigationTask]:
        """Get history of all completed and failed tasks."""
        return self.completed_tasks + self.failed_tasks
