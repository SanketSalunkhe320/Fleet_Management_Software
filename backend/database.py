# # database.py
# import sqlite3
# import json
# from datetime import datetime
# import logging

# logger = logging.getLogger(__name__)

# class DatabaseManager:
#     def __init__(self, db_path="amr_tasks.db"):
#         self.db_path = db_path
#         self.init_database()
    
#     def init_database(self):
#         """Initialize the database with required tables"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 # Tasks table with Z-axis support
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS tasks (
#                         id TEXT PRIMARY KEY,
#                         map_name TEXT NOT NULL,
#                         amr_name TEXT NOT NULL,
#                         status TEXT NOT NULL,
#                         z_axis REAL DEFAULT 0.0,
#                         assigned_robot TEXT,
#                         progress INTEGER DEFAULT 0,
#                         nodes_data TEXT NOT NULL,
#                         arrows_data TEXT NOT NULL,
#                         zones_data TEXT NOT NULL,
#                         created_at REAL,
#                         started_at REAL,
#                         completed_at REAL,
#                         FOREIGN KEY (assigned_robot) REFERENCES robots (id)
#                     )
#                 ''')
                
#                 # Robots table with Z-position support
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS robots (
#                         id TEXT PRIMARY KEY,
#                         name TEXT NOT NULL,
#                         status TEXT NOT NULL,
#                         battery_level REAL DEFAULT 100.0,
#                         position_x REAL DEFAULT 0.0,
#                         position_y REAL DEFAULT 0.0,
#                         position_theta REAL DEFAULT 0.0,
#                         z_position REAL DEFAULT 0.0,
#                         current_task TEXT,
#                         last_heartbeat REAL,
#                         created_at REAL DEFAULT (strftime('%s', 'now')),
#                         FOREIGN KEY (current_task) REFERENCES tasks (id)
#                     )
#                 ''')
                
#                 # Task execution history table
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS task_history (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         task_id TEXT NOT NULL,
#                         robot_id TEXT NOT NULL,
#                         status TEXT NOT NULL,
#                         message TEXT,
#                         progress INTEGER DEFAULT 0,
#                         timestamp REAL DEFAULT (strftime('%s', 'now')),
#                         FOREIGN KEY (task_id) REFERENCES tasks (id),
#                         FOREIGN KEY (robot_id) REFERENCES robots (id)
#                     )
#                 ''')
                
#                 # Map Nodes tables for different types
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS stations (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         name TEXT NOT NULL,
#                         x REAL NOT NULL,
#                         y REAL NOT NULL,
#                         z REAL DEFAULT 0.0,
#                         canvas_x REAL NOT NULL,
#                         canvas_y REAL NOT NULL,
#                         created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
#                         updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
#                     )
#                 ''')
                
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS docking (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         name TEXT NOT NULL,
#                         x REAL NOT NULL,
#                         y REAL NOT NULL,
#                         z REAL DEFAULT 0.0,
#                         canvas_x REAL NOT NULL,
#                         canvas_y REAL NOT NULL,
#                         created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
#                         updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
#                     )
#                 ''')
                
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS waypoints (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         name TEXT NOT NULL,
#                         x REAL NOT NULL,
#                         y REAL NOT NULL,
#                         z REAL DEFAULT 0.0,
#                         canvas_x REAL NOT NULL,
#                         canvas_y REAL NOT NULL,
#                         created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
#                         updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
#                     )
#                 ''')
                
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS charging (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         name TEXT NOT NULL,
#                         x REAL NOT NULL,
#                         y REAL NOT NULL,
#                         z REAL DEFAULT 0.0,
#                         canvas_x REAL NOT NULL,
#                         canvas_y REAL NOT NULL,
#                         created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
#                         updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
#                     )
#                 ''')
                
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS home (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         name TEXT NOT NULL,
#                         x REAL NOT NULL,
#                         y REAL NOT NULL,
#                         z REAL DEFAULT 0.0,
#                         canvas_x REAL NOT NULL,
#                         canvas_y REAL NOT NULL,
#                         created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
#                         updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
#                     )
#                 ''')
                
#                 # User table
#                 cursor.execute('''
#                     CREATE TABLE IF NOT EXISTS users (
#                         id INTEGER PRIMARY KEY AUTOINCREMENT,
#                         org_name TEXT NOT NULL,
#                         org_address TEXT NOT NULL,
#                         contact TEXT UNIQUE NOT NULL,
#                         password TEXT NOT NULL,
#                         created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
#                         is_active BOOLEAN DEFAULT TRUE
#                     )
#                 ''')
                
#                 # Create indexes for better performance
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_tasks_robot ON tasks(assigned_robot)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_robots_status ON robots(status)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_task_history_task_id ON task_history(task_id)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_task_history_timestamp ON task_history(timestamp)')
                
#                 # Indexes for map nodes
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_stations_coords ON stations(x, y)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_docking_coords ON docking(x, y)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_waypoints_coords ON waypoints(x, y)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_charging_coords ON charging(x, y)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_home_coords ON home(x, y)')
                
#                 # User indexes
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_users_contact ON users(contact)')
#                 cursor.execute('CREATE INDEX IF NOT EXISTS idx_users_org_name ON users(org_name)')
                
#                 conn.commit()
#                 logger.info("Database initialized successfully with all tables")
                
#         except Exception as e:
#             logger.error(f"Error initializing database: {e}")
#             raise

#     # ===== MAP NODES MANAGEMENT METHODS =====
    
#     def save_map_node(self, node_type, name, x, y, z=0.0, canvas_x=0.0, canvas_y=0.0):
#         """Save a map node to the corresponding table"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'''
#                     INSERT INTO {node_type} 
#                     (name, x, y, z, canvas_x, canvas_y)
#                     VALUES (?, ?, ?, ?, ?, ?)
#                 ''', (name, x, y, z, canvas_x, canvas_y))
                
#                 node_id = cursor.lastrowid
#                 conn.commit()
#                 logger.info(f"Saved {node_type} node: {name} at ({x}, {y}, {z})")
#                 return True, node_id
                
#         except Exception as e:
#             logger.error(f"Error saving {node_type} node: {e}")
#             return False, str(e)
    
#     def get_map_nodes(self, node_type):
#         """Get all nodes of a specific type"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'SELECT * FROM {node_type} ORDER BY created_at DESC')
#                 nodes = [dict(row) for row in cursor.fetchall()]
                
#                 logger.debug(f"Retrieved {len(nodes)} {node_type} from database")
#                 return nodes
                
#         except Exception as e:
#             logger.error(f"Error retrieving {node_type}: {e}")
#             return []
    
#     def get_all_map_nodes(self):
#         """Get all map nodes from all types"""
#         try:
#             all_nodes = {}
#             node_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
            
#             for node_type in node_types:
#                 all_nodes[node_type] = self.get_map_nodes(node_type)
            
#             return all_nodes
            
#         except Exception as e:
#             logger.error(f"Error retrieving all map nodes: {e}")
#             return {}
    
#     def update_map_node(self, node_type, node_id, name=None, x=None, y=None, z=None, canvas_x=None, canvas_y=None):
#         """Update a map node"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 query = f'UPDATE {node_type} SET updated_at = CURRENT_TIMESTAMP'
#                 params = []
                
#                 updates = []
#                 if name is not None:
#                     updates.append('name = ?')
#                     params.append(name)
#                 if x is not None:
#                     updates.append('x = ?')
#                     params.append(x)
#                 if y is not None:
#                     updates.append('y = ?')
#                     params.append(y)
#                 if z is not None:
#                     updates.append('z = ?')
#                     params.append(z)
#                 if canvas_x is not None:
#                     updates.append('canvas_x = ?')
#                     params.append(canvas_x)
#                 if canvas_y is not None:
#                     updates.append('canvas_y = ?')
#                     params.append(canvas_y)
                
#                 if not updates:
#                     return False  # Nothing to update
                
#                 query += ', ' + ', '.join(updates) + ' WHERE id = ?'
#                 params.append(node_id)
                
#                 cursor.execute(query, params)
#                 conn.commit()
#                 logger.info(f"Updated {node_type} node ID {node_id}")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error updating {node_type} node: {e}")
#             return False
    
#     def delete_map_node(self, node_type, node_id):
#         """Delete a map node"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'DELETE FROM {node_type} WHERE id = ?', (node_id,))
#                 conn.commit()
                
#                 logger.info(f"Deleted {node_type} node ID {node_id}")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error deleting {node_type} node: {e}")
#             return False
    
#     def get_map_node_by_id(self, node_type, node_id):
#         """Get a specific map node by ID"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'SELECT * FROM {node_type} WHERE id = ?', (node_id,))
#                 row = cursor.fetchone()
                
#                 if row:
#                     return dict(row)
#                 return None
                
#         except Exception as e:
#             logger.error(f"Error retrieving {node_type} node {node_id}: {e}")
#             return None
    
#     def get_map_nodes_count(self):
#         """Get count of nodes for each type"""
#         try:
#             counts = {}
#             node_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 for node_type in node_types:
#                     cursor.execute(f'SELECT COUNT(*) FROM {node_type}')
#                     counts[node_type] = cursor.fetchone()[0]
            
#             return counts
            
#         except Exception as e:
#             logger.error(f"Error getting map nodes count: {e}")
#             return {}
    
#     def search_map_nodes(self, node_type, search_term):
#         """Search map nodes by name"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'''
#                     SELECT * FROM {node_type} 
#                     WHERE name LIKE ? 
#                     ORDER BY created_at DESC
#                 ''', (f'%{search_term}%',))
                
#                 nodes = [dict(row) for row in cursor.fetchall()]
#                 return nodes
                
#         except Exception as e:
#             logger.error(f"Error searching {node_type}: {e}")
#             return []
    
#     def get_map_nodes_in_area(self, node_type, min_x, max_x, min_y, max_y):
#         """Get map nodes within a specific area"""
#         try:
#             valid_types = ['stations', 'docking', 'waypoints', 'charging', 'home']
#             if node_type not in valid_types:
#                 raise ValueError(f"Invalid node type. Must be one of: {valid_types}")
            
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute(f'''
#                     SELECT * FROM {node_type} 
#                     WHERE x BETWEEN ? AND ? AND y BETWEEN ? AND ?
#                     ORDER BY created_at DESC
#                 ''', (min_x, max_x, min_y, max_y))
                
#                 nodes = [dict(row) for row in cursor.fetchall()]
#                 return nodes
                
#         except Exception as e:
#             logger.error(f"Error getting {node_type} in area: {e}")
#             return []

#     # ===== TASK MANAGEMENT METHODS =====
    
#     def save_task(self, task_data):
#         """Save a task to the database"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     INSERT OR REPLACE INTO tasks 
#                     (id, map_name, amr_name, status, z_axis, assigned_robot, progress, 
#                      nodes_data, arrows_data, zones_data, created_at, started_at, completed_at)
#                     VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
#                 ''', (
#                     task_data['id'],
#                     task_data['map_name'],
#                     task_data['amr_name'],
#                     task_data['status'],
#                     task_data.get('z_axis', 0.0),
#                     task_data.get('assigned_robot'),
#                     task_data.get('progress', 0),
#                     json.dumps(task_data['nodes']),
#                     json.dumps(task_data['arrows']),
#                     json.dumps(task_data['zones']),
#                     task_data.get('created_at'),
#                     task_data.get('started_at'),
#                     task_data.get('completed_at')
#                 ))
                
#                 conn.commit()
#                 logger.info(f"Task {task_data['id']} saved to database with Z-axis: {task_data.get('z_axis', 0.0)}")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error saving task: {e}")
#             return False

#     def update_task_status(self, task_id, status=None, progress=None, assigned_robot=None, 
#                           started_at=None, completed_at=None, z_axis=None):
#         """Update task status and progress"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 query = 'UPDATE tasks SET '
#                 params = []
#                 updates = []
                
#                 if status is not None:
#                     updates.append('status = ?')
#                     params.append(status)
                
#                 if progress is not None:
#                     updates.append('progress = ?')
#                     params.append(progress)
                
#                 if assigned_robot is not None:
#                     updates.append('assigned_robot = ?')
#                     params.append(assigned_robot)
                
#                 if started_at is not None:
#                     updates.append('started_at = ?')
#                     params.append(started_at)
                
#                 if completed_at is not None:
#                     updates.append('completed_at = ?')
#                     params.append(completed_at)
                
#                 if z_axis is not None:
#                     updates.append('z_axis = ?')
#                     params.append(z_axis)
                
#                 if not updates:
#                     return False  # Nothing to update
                
#                 query += ', '.join(updates) + ' WHERE id = ?'
#                 params.append(task_id)
                
#                 cursor.execute(query, params)
#                 conn.commit()
#                 logger.info(f"Task {task_id} updated - status: {status}, progress: {progress}, Z-axis: {z_axis}")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error updating task status: {e}")
#             return False

#     def get_all_tasks(self):
#         """Get all tasks from database"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     SELECT * FROM tasks ORDER BY created_at DESC
#                 ''')
#                 tasks = []
#                 for row in cursor.fetchall():
#                     task = dict(row)
#                     # Parse JSON data
#                     task['nodes'] = json.loads(task['nodes_data'])
#                     task['arrows'] = json.loads(task['arrows_data'])
#                     task['zones'] = json.loads(task['zones_data'])
#                     tasks.append(task)
                
#                 return tasks
#         except Exception as e:
#             logger.error(f"Error retrieving tasks: {e}")
#             return []

#     def get_task_by_id(self, task_id):
#         """Get a specific task by ID"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM tasks WHERE id = ?', (task_id,))
#                 row = cursor.fetchone()
                
#                 if row:
#                     task = dict(row)
#                     # Parse JSON data
#                     task['nodes'] = json.loads(task['nodes_data'])
#                     task['arrows'] = json.loads(task['arrows_data'])
#                     task['zones'] = json.loads(task['zones_data'])
#                     return task
#                 return None
#         except Exception as e:
#             logger.error(f"Error retrieving task {task_id}: {e}")
#             return None

#     def get_tasks_by_status(self, status):
#         """Get tasks by status"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM tasks WHERE status = ? ORDER BY created_at DESC', (status,))
#                 tasks = []
#                 for row in cursor.fetchall():
#                     task = dict(row)
#                     # Parse JSON data
#                     task['nodes'] = json.loads(task['nodes_data'])
#                     task['arrows'] = json.loads(task['arrows_data'])
#                     task['zones'] = json.loads(task['zones_data'])
#                     tasks.append(task)
                
#                 return tasks
#         except Exception as e:
#             logger.error(f"Error retrieving tasks with status {status}: {e}")
#             return []

#     def delete_task(self, task_id):
#         """Delete a task"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('DELETE FROM tasks WHERE id = ?', (task_id,))
#                 conn.commit()
                
#                 logger.info(f"Deleted task {task_id}")
#                 return True
#         except Exception as e:
#             logger.error(f"Error deleting task {task_id}: {e}")
#             return False

#     # ===== ROBOT MANAGEMENT METHODS =====

#     def save_robot(self, robot_data):
#         """Save or update robot information"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     INSERT OR REPLACE INTO robots 
#                     (id, name, status, battery_level, position_x, position_y, position_theta, 
#                      z_position, current_task, last_heartbeat)
#                     VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
#                 ''', (
#                     robot_data['id'],
#                     robot_data['name'],
#                     robot_data['status'],
#                     robot_data.get('battery_level', 100.0),
#                     robot_data.get('position', {}).get('x', 0.0),
#                     robot_data.get('position', {}).get('y', 0.0),
#                     robot_data.get('position', {}).get('theta', 0.0),
#                     robot_data.get('z_position', 0.0),
#                     robot_data.get('current_task'),
#                     robot_data.get('last_heartbeat')
#                 ))
                
#                 conn.commit()
#                 logger.info(f"Robot {robot_data['id']} saved to database with Z-position: {robot_data.get('z_position', 0.0)}")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error saving robot: {e}")
#             return False

#     def update_robot_status(self, robot_id, status=None, battery_level=None, position=None, 
#                            z_position=None, current_task=None, last_heartbeat=None):
#         """Update robot status and information"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 query = 'UPDATE robots SET '
#                 params = []
#                 updates = []
                
#                 if status is not None:
#                     updates.append('status = ?')
#                     params.append(status)
                
#                 if battery_level is not None:
#                     updates.append('battery_level = ?')
#                     params.append(battery_level)
                
#                 if position is not None:
#                     updates.append('position_x = ?')
#                     params.append(position.get('x', 0.0))
#                     updates.append('position_y = ?')
#                     params.append(position.get('y', 0.0))
#                     updates.append('position_theta = ?')
#                     params.append(position.get('theta', 0.0))
                
#                 if z_position is not None:
#                     updates.append('z_position = ?')
#                     params.append(z_position)
                
#                 if current_task is not None:
#                     updates.append('current_task = ?')
#                     params.append(current_task)
                
#                 if last_heartbeat is not None:
#                     updates.append('last_heartbeat = ?')
#                     params.append(last_heartbeat)
                
#                 if not updates:
#                     return False  # Nothing to update
                
#                 query += ', '.join(updates) + ' WHERE id = ?'
#                 params.append(robot_id)
                
#                 cursor.execute(query, params)
#                 conn.commit()
#                 logger.info(f"Robot {robot_id} updated")
#                 return True
                
#         except Exception as e:
#             logger.error(f"Error updating robot status: {e}")
#             return False

#     def get_all_robots(self):
#         """Get all robots from database"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM robots ORDER BY created_at DESC')
#                 robots = [dict(row) for row in cursor.fetchall()]
                
#                 return robots
#         except Exception as e:
#             logger.error(f"Error retrieving robots: {e}")
#             return []

#     def get_robot_by_id(self, robot_id):
#         """Get a specific robot by ID"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM robots WHERE id = ?', (robot_id,))
#                 row = cursor.fetchone()
                
#                 if row:
#                     return dict(row)
#                 return None
#         except Exception as e:
#             logger.error(f"Error retrieving robot {robot_id}: {e}")
#             return None

#     def get_robots_by_status(self, status):
#         """Get robots by status"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM robots WHERE status = ? ORDER BY created_at DESC', (status,))
#                 robots = [dict(row) for row in cursor.fetchall()]
                
#                 return robots
#         except Exception as e:
#             logger.error(f"Error retrieving robots with status {status}: {e}")
#             return []

#     def delete_robot(self, robot_id):
#         """Delete a robot"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('DELETE FROM robots WHERE id = ?', (robot_id,))
#                 conn.commit()
                
#                 logger.info(f"Deleted robot {robot_id}")
#                 return True
#         except Exception as e:
#             logger.error(f"Error deleting robot {robot_id}: {e}")
#             return False

#     # ===== TASK HISTORY METHODS =====

#     def add_task_history(self, task_id, robot_id, status, message=None, progress=None):
#         """Add entry to task history"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     INSERT INTO task_history 
#                     (task_id, robot_id, status, message, progress)
#                     VALUES (?, ?, ?, ?, ?)
#                 ''', (task_id, robot_id, status, message, progress))
                
#                 conn.commit()
#                 logger.info(f"Added task history for task {task_id}, robot {robot_id}, status {status}")
#                 return True
#         except Exception as e:
#             logger.error(f"Error adding task history: {e}")
#             return False

#     def get_task_history(self, task_id=None, robot_id=None, limit=100):
#         """Get task history"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 query = 'SELECT * FROM task_history'
#                 params = []
                
#                 conditions = []
#                 if task_id is not None:
#                     conditions.append('task_id = ?')
#                     params.append(task_id)
#                 if robot_id is not None:
#                     conditions.append('robot_id = ?')
#                     params.append(robot_id)
                
#                 if conditions:
#                     query += ' WHERE ' + ' AND '.join(conditions)
                
#                 query += ' ORDER BY timestamp DESC LIMIT ?'
#                 params.append(limit)
                
#                 cursor.execute(query, params)
#                 history = [dict(row) for row in cursor.fetchall()]
                
#                 return history
#         except Exception as e:
#             logger.error(f"Error retrieving task history: {e}")
#             return []

#     def get_robot_tasks(self, robot_id):
#         """Get tasks assigned to a specific robot"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     SELECT * FROM tasks 
#                     WHERE assigned_robot = ? 
#                     ORDER BY created_at DESC
#                 ''', (robot_id,))
                
#                 tasks = []
#                 for row in cursor.fetchall():
#                     task = dict(row)
#                     # Parse JSON data
#                     task['nodes'] = json.loads(task['nodes_data'])
#                     task['arrows'] = json.loads(task['arrows_data'])
#                     task['zones'] = json.loads(task['zones_data'])
#                     tasks.append(task)
                
#                 return tasks
#         except Exception as e:
#             logger.error(f"Error retrieving tasks for robot {robot_id}: {e}")
#             return []

#     # ===== USER MANAGEMENT METHODS =====

#     def create_user(self, org_name, org_address, contact, password):
#         """Create a new user"""
#         try:
#             import hashlib
#             hashed_password = hashlib.sha256(password.encode()).hexdigest()
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     INSERT INTO users (org_name, org_address, contact, password)
#                     VALUES (?, ?, ?, ?)
#                 ''', (org_name, org_address, contact, hashed_password))
                
#                 conn.commit()
#                 user_id = cursor.lastrowid
#                 logger.info(f"User created: {org_name} with contact {contact}")
#                 return True, user_id
                
#         except sqlite3.IntegrityError:
#             return False, "Contact already exists"
#         except Exception as e:
#             logger.error(f"Error creating user: {e}")
#             return False, str(e)

#     def authenticate_user(self, username, password):
#         """Authenticate user"""
#         try:
#             import hashlib
#             hashed_password = hashlib.sha256(password.encode()).hexdigest()
            
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('''
#                     SELECT * FROM users 
#                     WHERE (org_name = ? OR contact = ?) AND password = ? AND is_active = TRUE
#                 ''', (username, username, hashed_password))
                
#                 user = cursor.fetchone()
                
#                 if user:
#                     user_data = {
#                         'id': user['id'],
#                         'orgName': user['org_name'],
#                         'orgAddress': user['org_address'],
#                         'contact': user['contact'],
#                         'createdAt': user['created_at']
#                     }
#                     return True, user_data
#                 else:
#                     return False, "Invalid credentials"
                    
#         except Exception as e:
#             logger.error(f"Error authenticating user: {e}")
#             return False, str(e)

#     def user_exists(self, contact):
#         """Check if user exists"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT id FROM users WHERE contact = ?', (contact,))
#                 user = cursor.fetchone()
                
#                 return user is not None
#         except Exception as e:
#             logger.error(f"Error checking user existence: {e}")
#             return False

#     def get_all_users(self):
#         """Get all users (for admin purposes)"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 conn.row_factory = sqlite3.Row
#                 cursor = conn.cursor()
                
#                 cursor.execute('SELECT * FROM users ORDER BY created_at DESC')
#                 users = [dict(row) for row in cursor.fetchall()]
                
#                 return users
#         except Exception as e:
#             logger.error(f"Error retrieving users: {e}")
#             return []

#     # ===== DATABASE UTILITY METHODS =====

#     def cleanup_old_data(self, days_old=30):
#         """Clean up old data"""
#         try:
#             cutoff_time = datetime.now().timestamp() - (days_old * 24 * 60 * 60)
            
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 # Clean up old task history
#                 cursor.execute('DELETE FROM task_history WHERE timestamp < ?', (cutoff_time,))
#                 history_deleted = cursor.rowcount
                
#                 # Clean up completed tasks older than cutoff
#                 cursor.execute('DELETE FROM tasks WHERE completed_at IS NOT NULL AND completed_at < ?', (cutoff_time,))
#                 tasks_deleted = cursor.rowcount
                
#                 conn.commit()
                
#                 logger.info(f"Cleaned up {history_deleted} history records and {tasks_deleted} old tasks")
#                 return True, f"Cleaned up {history_deleted} history records and {tasks_deleted} old tasks"
                
#         except Exception as e:
#             logger.error(f"Error cleaning up old data: {e}")
#             return False, str(e)

#     def get_database_stats(self):
#         """Get comprehensive database statistics"""
#         try:
#             with sqlite3.connect(self.db_path) as conn:
#                 cursor = conn.cursor()
                
#                 # Get all table names
#                 cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
#                 tables = [row[0] for row in cursor.fetchall()]
                
#                 # Get row counts for each table
#                 table_counts = {}
#                 total_records = 0
                
#                 for table in tables:
#                     cursor.execute(f"SELECT COUNT(*) FROM {table}")
#                     count = cursor.fetchone()[0]
#                     table_counts[table] = count
#                     total_records += count
                
#                 return {
#                     "table_counts": table_counts,
#                     "total_records": total_records,
#                     "tables": tables
#                 }
                
#         except Exception as e:
#             logger.error(f"Error getting database stats: {e}")
#             return {"table_counts": {}, "total_records": 0, "tables": []}

#     def backup_database(self, backup_path):
#         """Create a backup of the database"""
#         try:
#             import shutil
#             shutil.copy2(self.db_path, backup_path)
#             logger.info(f"Database backed up to {backup_path}")
#             return True
#         except Exception as e:
#             logger.error(f"Error backing up database: {e}")
#             return False

# # Create a global instance for easy import
# db_manager = DatabaseManager()