# from flask import Flask, request, jsonify
# import sqlite3
# import json
# from datetime import datetime
# import os
# import logging
# from flask_cors import CORS

# # Configure logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# app = Flask(__name__)
# CORS(app)  # Enable CORS for all routes

# # Database configuration
# DATABASE = 'amr_tasks.db'

# def get_db_connection():
#     """Get database connection with error handling"""
#     try:
#         conn = sqlite3.connect(DATABASE)
#         conn.row_factory = sqlite3.Row
#         return conn
#     except sqlite3.Error as e:
#         logger.error(f"Database connection error: {e}")
#         raise

# def init_database():
#     """Initialize database with separate tables for each node type"""
#     try:
#         conn = get_db_connection()
        
#         # Create separate tables for each node type
#         tables = {
#             'stations': '''
#                 CREATE TABLE IF NOT EXISTS stations (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     label TEXT,
#                     node_type TEXT NOT NULL DEFAULT 'station',
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'docking_points': '''
#                 CREATE TABLE IF NOT EXISTS docking_points (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     label TEXT,
#                     node_type TEXT NOT NULL DEFAULT 'docking',
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'waypoints': '''
#                 CREATE TABLE IF NOT EXISTS waypoints (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     label TEXT,
#                     node_type TEXT NOT NULL DEFAULT 'waypoint',
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'home_positions': '''
#                 CREATE TABLE IF NOT EXISTS home_positions (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     label TEXT,
#                     node_type TEXT NOT NULL DEFAULT 'home',
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'charging_stations': '''
#                 CREATE TABLE IF NOT EXISTS charging_stations (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     label TEXT,
#                     node_type TEXT NOT NULL DEFAULT 'charging',
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'arrows': '''
#                 CREATE TABLE IF NOT EXISTS arrows (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     from_node_id TEXT NOT NULL,
#                     to_node_id TEXT NOT NULL,
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             ''',
#             'zones': '''
#                 CREATE TABLE IF NOT EXISTS zones (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     points_json TEXT NOT NULL,
#                     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
#                 )
#             '''
#         }
        
#         for table_name, create_sql in tables.items():
#             conn.execute(create_sql)
#             logger.info(f"✅ Table '{table_name}' initialized")
        
#         conn.commit()
#         conn.close()
#         logger.info("✅ Database initialization completed successfully")
        
#     except Exception as e:
#         logger.error(f"❌ Database initialization failed: {e}")
#         raise

# @app.route('/')
# def index():
#     """Root endpoint with API information"""
#     return jsonify({
#         'status': 'success',
#         'message': 'AMR Map Editor API',
#         'endpoints': {
#             'save_nodes': 'POST /save-nodes',
#             'load_nodes': 'GET /load-nodes',
#             'clear_nodes': 'DELETE /clear-nodes',
#             'database_info': 'GET /database-info',
#             'health': 'GET /health'
#         },
#         'version': '1.0.0'
#     })

# @app.route('/health')
# def health_check():
#     """Health check endpoint"""
#     try:
#         conn = get_db_connection()
#         conn.execute('SELECT 1')
#         conn.close()
#         return jsonify({
#             'status': 'success',
#             'message': 'API and database are healthy',
#             'timestamp': datetime.now().isoformat()
#         })
#     except Exception as e:
#         return jsonify({
#             'status': 'error',
#             'message': f'Health check failed: {str(e)}'
#         }), 500

# @app.route('/save-nodes', methods=['POST'])
# def save_nodes():
#     """Save nodes, arrows, and zones to database"""
#     try:
#         data = request.get_json()
#         if not data:
#             return jsonify({
#                 'status': 'error',
#                 'message': 'No JSON data provided'
#             }), 400
        
#         map_name = data.get('mapName', 'default')
#         nodes = data.get('nodes', [])
#         arrows = data.get('arrows', [])
#         zones = data.get('zones', [])
        
#         logger.info(f"💾 Saving {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones for map '{map_name}'")
        
#         conn = get_db_connection()
#         saved_counts = {
#             'stations': 0,
#             'docking_points': 0,
#             'waypoints': 0,
#             'home_positions': 0,
#             'charging_stations': 0,
#             'arrows': 0,
#             'zones': 0
#         }
        
#         # Clear existing data for this map
#         tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#         for table in tables_to_clear:
#             result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
#             logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
#         # Save nodes to respective tables
#         for node in nodes:
#             table_name = None
#             if node.get('type') == 'station':
#                 table_name = 'stations'
#                 saved_counts['stations'] += 1
#             elif node.get('type') == 'docking':
#                 table_name = 'docking_points'
#                 saved_counts['docking_points'] += 1
#             elif node.get('type') == 'waypoint':
#                 table_name = 'waypoints'
#                 saved_counts['waypoints'] += 1
#             elif node.get('type') == 'home':
#                 table_name = 'home_positions'
#                 saved_counts['home_positions'] += 1
#             elif node.get('type') == 'charging':
#                 table_name = 'charging_stations'
#                 saved_counts['charging_stations'] += 1
            
#             if table_name:
#                 conn.execute(
#                     f'INSERT INTO {table_name} (map_name, name, x, y, label, node_type) VALUES (?, ?, ?, ?, ?, ?)',
#                     (map_name, node.get('label', ''), node.get('rosX', 0), node.get('rosY', 0), 
#                      node.get('label', ''), node.get('type', ''))
#                 )
        
#         # Save arrows
#         for arrow in arrows:
#             conn.execute(
#                 'INSERT INTO arrows (map_name, from_node_id, to_node_id) VALUES (?, ?, ?)',
#                 (map_name, arrow.get('fromId', ''), arrow.get('toId', ''))
#             )
#             saved_counts['arrows'] += 1
        
#         # Save zones
#         for zone in zones:
#             conn.execute(
#                 'INSERT INTO zones (map_name, name, points_json) VALUES (?, ?, ?)',
#                 (map_name, zone.get('name', ''), json.dumps(zone.get('points', [])))
#             )
#             saved_counts['zones'] += 1
        
#         conn.commit()
#         conn.close()
        
#         total_records = sum(saved_counts.values())
#         logger.info(f"✅ Successfully saved {total_records} records for map '{map_name}'")
        
#         return jsonify({
#             'status': 'success',
#             'message': f'Saved {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones',
#             'map_name': map_name,
#             'database': DATABASE,
#             'saved_counts': saved_counts,
#             'total_records': total_records,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error saving nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to save nodes: {str(e)}'
#         }), 500

# @app.route('/load-nodes', methods=['GET'])
# def load_nodes():
#     """Load nodes, arrows, and zones from database"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"📂 Loading data for map '{map_name}'")
        
#         conn = get_db_connection()
        
#         # Load nodes from all tables
#         nodes = []
        
#         # Load from each node table
#         node_tables = {
#             'stations': 'station',
#             'docking_points': 'docking',
#             'waypoints': 'waypoint',
#             'home_positions': 'home',
#             'charging_stations': 'charging'
#         }
        
#         for table_name, node_type in node_tables.items():
#             rows = conn.execute(
#                 f'SELECT * FROM {table_name} WHERE map_name = ?', (map_name,)
#             ).fetchall()
            
#             for row in rows:
#                 nodes.append({
#                     'id': f"node_{row['id']}",
#                     'type': node_type,
#                     'name': row['name'],
#                     'x': row['x'],
#                     'y': row['y'],
#                     'label': row['label'],
#                     'rosX': row['x'],  # For compatibility with frontend
#                     'rosY': row['y'],  # For compatibility with frontend
#                     'dbId': row['id']  # Keep original database ID
#                 })
        
#         # Load arrows
#         arrows = []
#         arrow_rows = conn.execute(
#             'SELECT * FROM arrows WHERE map_name = ?', (map_name,)
#         ).fetchall()
        
#         for row in arrow_rows:
#             arrows.append({
#                 'id': row['id'],
#                 'fromId': row['from_node_id'],
#                 'toId': row['to_node_id']
#             })
        
#         # Load zones
#         zones = []
#         zone_rows = conn.execute(
#             'SELECT * FROM zones WHERE map_name = ?', (map_name,)
#         ).fetchall()
        
#         for row in zone_rows:
#             try:
#                 points_data = json.loads(row['points_json'])
#                 zones.append({
#                     'id': row['id'],
#                     'name': row['name'],
#                     'points': points_data
#                 })
#             except json.JSONDecodeError:
#                 logger.warning(f"Invalid JSON in zone {row['id']}, skipping")
#                 continue
        
#         conn.close()
        
#         logger.info(f"✅ Loaded {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones")
        
#         return jsonify({
#             'status': 'success',
#             'map_name': map_name,
#             'database': DATABASE,
#             'nodes': nodes,
#             'arrows': arrows,
#             'zones': zones,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error loading nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to load nodes: {str(e)}'
#         }), 500

# @app.route('/clear-nodes', methods=['DELETE'])
# def clear_nodes():
#     """Clear all nodes, arrows, and zones for a specific map"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"🗑️ Clearing data for map '{map_name}'")
        
#         conn = get_db_connection()
        
#         tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#         cleared_counts = {}
        
#         for table in tables_to_clear:
#             result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
#             cleared_counts[table] = result.rowcount
#             logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
#         conn.commit()
#         conn.close()
        
#         total_cleared = sum(cleared_counts.values())
#         logger.info(f"✅ Successfully cleared {total_cleared} records")
        
#         return jsonify({
#             'status': 'success',
#             'message': f'Cleared {total_cleared} records from {map_name}',
#             'map_name': map_name,
#             'cleared_counts': cleared_counts,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error clearing nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to clear nodes: {str(e)}'
#         }), 500

# @app.route('/database-info', methods=['GET'])
# def database_info():
#     """Get database information and statistics"""
#     try:
#         conn = get_db_connection()
        
#         # Get database size
#         db_size = os.path.getsize(DATABASE) / (1024 * 1024)  # MB
        
#         # Get table counts
#         tables = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#         table_counts = {}
#         total_records = 0
        
#         for table in tables:
#             count = conn.execute(f'SELECT COUNT(*) FROM {table}').fetchone()[0]
#             table_counts[table] = count
#             total_records += count
        
#         # Get maps in database
#         maps = conn.execute('''
#             SELECT map_name, COUNT(*) as count FROM (
#                 SELECT map_name FROM stations UNION ALL
#                 SELECT map_name FROM docking_points UNION ALL
#                 SELECT map_name FROM waypoints UNION ALL
#                 SELECT map_name FROM home_positions UNION ALL
#                 SELECT map_name FROM charging_stations UNION ALL
#                 SELECT map_name FROM arrows UNION ALL
#                 SELECT map_name FROM zones
#             ) GROUP BY map_name
#         ''').fetchall()
        
#         conn.close()
        
#         return jsonify({
#             'status': 'success',
#             'database_name': DATABASE,
#             'database_size_mb': round(db_size, 2),
#             'total_records': total_records,
#             'table_counts': table_counts,
#             'maps': [dict(row) for row in maps],
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error getting database info: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to get database info: {str(e)}'
#         }), 500

# @app.errorhandler(404)
# def not_found(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Endpoint not found'
#     }), 404

# @app.errorhandler(500)
# def internal_error(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Internal server error'
#     }), 500

# if __name__ == '__main__':
#     print("🚀 Starting AMR Map Editor API Server...")
#     print("📦 Initializing database...")
#     init_database()
#     print("✅ Database initialized successfully!")
#     print("🗂️ Available tables:")
#     print("   🏭 Stations table")
#     print("   📍 Docking points table") 
#     print("   🚩 Waypoints table")
#     print("   🏠 Home positions table")
#     print("   🔋 Charging stations table")
#     print("   ➡️ Arrows table")
#     print("   🗺️ Zones table")
#     print("\n🌐 Starting Flask server...")
#     print("   📍 Local: http://localhost:5000")
#     print("   🌍 Network: http://0.0.0.0:5000")
#     print("\n🔍 Testing connection...")
    
#     # Test the health endpoint on startup
#     try:
#         with app.test_client() as client:
#             response = client.get('/health')
#             if response.get_json()['status'] == 'success':
#                 print("✅ Health check passed!")
#             else:
#                 print("❌ Health check failed!")
#     except Exception as e:
#         print(f"❌ Health check failed: {e}")
    
#     # Run on all interfaces (0.0.0.0) instead of just localhost
#     app.run(debug=True, host='0.0.0.0', port=5000)



from flask import Flask, request, jsonify
import sqlite3
import json
from datetime import datetime
import os
import logging
from flask_cors import CORS
import re

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Database base directory
DATABASE_DIR = 'map_databases'

def sanitize_map_name(map_name):
    """Sanitize map name for use as filename"""
    # Remove invalid filename characters
    sanitized = re.sub(r'[^\w\-_\. ]', '', map_name)
    # Replace spaces with underscores
    sanitized = sanitized.replace(' ', '_')
    # Ensure it's not empty
    if not sanitized:
        sanitized = 'default_map'
    return sanitized

def get_database_path(map_name):
    """Get database path for a specific map"""
    # Create database directory if it doesn't exist
    if not os.path.exists(DATABASE_DIR):
        os.makedirs(DATABASE_DIR)
    
    # Sanitize map name for filename
    safe_map_name = sanitize_map_name(map_name)
    database_name = f"{safe_map_name}.db"
    return os.path.join(DATABASE_DIR, database_name)

def get_db_connection(map_name):
    """Get database connection for a specific map with error handling"""
    try:
        db_path = get_database_path(map_name)
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        return conn
    except sqlite3.Error as e:
        logger.error(f"Database connection error for map '{map_name}': {e}")
        raise

def init_database_for_map(map_name):
    """Initialize database for a specific map"""
    try:
        conn = get_db_connection(map_name)
        
        # Create separate tables for each node type
        tables = {
            'stations': '''
                CREATE TABLE IF NOT EXISTS stations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    label TEXT,
                    node_type TEXT NOT NULL DEFAULT 'station',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'docking_points': '''
                CREATE TABLE IF NOT EXISTS docking_points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    label TEXT,
                    node_type TEXT NOT NULL DEFAULT 'docking',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'waypoints': '''
                CREATE TABLE IF NOT EXISTS waypoints (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    label TEXT,
                    node_type TEXT NOT NULL DEFAULT 'waypoint',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'home_positions': '''
                CREATE TABLE IF NOT EXISTS home_positions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    label TEXT,
                    node_type TEXT NOT NULL DEFAULT 'home',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'charging_stations': '''
                CREATE TABLE IF NOT EXISTS charging_stations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    label TEXT,
                    node_type TEXT NOT NULL DEFAULT 'charging',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'arrows': '''
                CREATE TABLE IF NOT EXISTS arrows (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    from_node_id TEXT NOT NULL,
                    to_node_id TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''',
            'zones': '''
                CREATE TABLE IF NOT EXISTS zones (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    points_json TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            '''
        }
        
        for table_name, create_sql in tables.items():
            conn.execute(create_sql)
            logger.info(f"✅ Table '{table_name}' initialized for map '{map_name}'")
        
        conn.commit()
        conn.close()
        logger.info(f"✅ Database initialization completed for map '{map_name}'")
        
    except Exception as e:
        logger.error(f"❌ Database initialization failed for map '{map_name}': {e}")
        raise

@app.route('/')
def index():
    """Root endpoint with API information"""
    return jsonify({
        'status': 'success',
        'message': 'AMR Map Editor API',
        'endpoints': {
            'save_nodes': 'POST /save-nodes',
            'load_nodes': 'GET /load-nodes',
            'clear_nodes': 'DELETE /clear-nodes',
            'database_info': 'GET /database-info',
            'health': 'GET /health',
            'list_maps': 'GET /list-maps'
        },
        'version': '1.0.0',
        'note': 'Each map has its own database file'
    })

@app.route('/health')
def health_check():
    """Health check endpoint"""
    try:
        # Test with default map
        db_path = get_database_path('default')
        if not os.path.exists(db_path):
            init_database_for_map('default')
        
        conn = sqlite3.connect(db_path)
        conn.execute('SELECT 1')
        conn.close()
        return jsonify({
            'status': 'success',
            'message': 'API is healthy',
            'database_system': 'per-map databases',
            'database_directory': DATABASE_DIR,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'Health check failed: {str(e)}'
        }), 500

@app.route('/save-nodes', methods=['POST'])
def save_nodes():
    """Save nodes, arrows, and zones to database"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({
                'status': 'error',
                'message': 'No JSON data provided'
            }), 400
        
        map_name = data.get('mapName', 'default')
        
        # Initialize database for this map if it doesn't exist
        db_path = get_database_path(map_name)
        if not os.path.exists(db_path):
            init_database_for_map(map_name)
        
        nodes = data.get('nodes', [])
        arrows = data.get('arrows', [])
        zones = data.get('zones', [])
        
        logger.info(f"💾 Saving {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones for map '{map_name}'")
        
        conn = get_db_connection(map_name)
        saved_counts = {
            'stations': 0,
            'docking_points': 0,
            'waypoints': 0,
            'home_positions': 0,
            'charging_stations': 0,
            'arrows': 0,
            'zones': 0
        }
        
        # Clear existing data for this map
        tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
        for table in tables_to_clear:
            result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
            logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
        # Save nodes to respective tables
        for node in nodes:
            table_name = None
            if node.get('type') == 'station':
                table_name = 'stations'
                saved_counts['stations'] += 1
            elif node.get('type') == 'docking':
                table_name = 'docking_points'
                saved_counts['docking_points'] += 1
            elif node.get('type') == 'waypoint':
                table_name = 'waypoints'
                saved_counts['waypoints'] += 1
            elif node.get('type') == 'home':
                table_name = 'home_positions'
                saved_counts['home_positions'] += 1
            elif node.get('type') == 'charging':
                table_name = 'charging_stations'
                saved_counts['charging_stations'] += 1
            
            if table_name:
                conn.execute(
                    f'INSERT INTO {table_name} (map_name, name, x, y, label, node_type) VALUES (?, ?, ?, ?, ?, ?)',
                    (map_name, node.get('label', ''), node.get('rosX', 0), node.get('rosY', 0), 
                     node.get('label', ''), node.get('type', ''))
                )
        
        # Save arrows
        for arrow in arrows:
            conn.execute(
                'INSERT INTO arrows (map_name, from_node_id, to_node_id) VALUES (?, ?, ?)',
                (map_name, arrow.get('fromId', ''), arrow.get('toId', ''))
            )
            saved_counts['arrows'] += 1
        
        # Save zones
        for zone in zones:
            conn.execute(
                'INSERT INTO zones (map_name, name, points_json) VALUES (?, ?, ?)',
                (map_name, zone.get('name', ''), json.dumps(zone.get('points', [])))
            )
            saved_counts['zones'] += 1
        
        conn.commit()
        conn.close()
        
        total_records = sum(saved_counts.values())
        logger.info(f"✅ Successfully saved {total_records} records for map '{map_name}'")
        
        return jsonify({
            'status': 'success',
            'message': f'Saved {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones',
            'map_name': map_name,
            'database_file': os.path.basename(db_path),
            'database_path': db_path,
            'saved_counts': saved_counts,
            'total_records': total_records,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error saving nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to save nodes: {str(e)}'
        }), 500

@app.route('/load-nodes', methods=['GET'])
def load_nodes():
    """Load nodes, arrows, and zones from database"""
    try:
        map_name = request.args.get('map_name', 'default')
        logger.info(f"📂 Loading data for map '{map_name}'")
        
        db_path = get_database_path(map_name)
        
        # Check if database exists
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}',
                'suggestion': 'Save nodes first to create database'
            }), 404
        
        conn = get_db_connection(map_name)
        
        # Load nodes from all tables
        nodes = []
        
        # Load from each node table
        node_tables = {
            'stations': 'station',
            'docking_points': 'docking',
            'waypoints': 'waypoint',
            'home_positions': 'home',
            'charging_stations': 'charging'
        }
        
        for table_name, node_type in node_tables.items():
            rows = conn.execute(
                f'SELECT * FROM {table_name} WHERE map_name = ?', (map_name,)
            ).fetchall()
            
            for row in rows:
                nodes.append({
                    'id': f"node_{row['id']}",
                    'type': node_type,
                    'name': row['name'],
                    'x': row['x'],
                    'y': row['y'],
                    'label': row['label'],
                    'rosX': row['x'],  # For compatibility with frontend
                    'rosY': row['y'],  # For compatibility with frontend
                    'dbId': row['id']  # Keep original database ID
                })
        
        # Load arrows
        arrows = []
        arrow_rows = conn.execute(
            'SELECT * FROM arrows WHERE map_name = ?', (map_name,)
        ).fetchall()
        
        for row in arrow_rows:
            arrows.append({
                'id': row['id'],
                'fromId': row['from_node_id'],
                'toId': row['to_node_id']
            })
        
        # Load zones
        zones = []
        zone_rows = conn.execute(
            'SELECT * FROM zones WHERE map_name = ?', (map_name,)
        ).fetchall()
        
        for row in zone_rows:
            try:
                points_data = json.loads(row['points_json'])
                zones.append({
                    'id': row['id'],
                    'name': row['name'],
                    'points': points_data
                })
            except json.JSONDecodeError:
                logger.warning(f"Invalid JSON in zone {row['id']}, skipping")
                continue
        
        conn.close()
        
        logger.info(f"✅ Loaded {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones")
        
        return jsonify({
            'status': 'success',
            'map_name': map_name,
            'database_file': os.path.basename(db_path),
            'nodes': nodes,
            'arrows': arrows,
            'zones': zones,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error loading nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to load nodes: {str(e)}'
        }), 500

@app.route('/clear-nodes', methods=['DELETE'])
def clear_nodes():
    """Clear all nodes, arrows, and zones for a specific map"""
    try:
        map_name = request.args.get('map_name', 'default')
        logger.info(f"🗑️ Clearing data for map '{map_name}'")
        
        db_path = get_database_path(map_name)
        
        # Check if database exists
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}',
                'suggestion': 'Save nodes first to create database'
            }), 404
        
        conn = get_db_connection(map_name)
        
        tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
        cleared_counts = {}
        
        for table in tables_to_clear:
            result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
            cleared_counts[table] = result.rowcount
            logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
        conn.commit()
        conn.close()
        
        total_cleared = sum(cleared_counts.values())
        logger.info(f"✅ Successfully cleared {total_cleared} records")
        
        return jsonify({
            'status': 'success',
            'message': f'Cleared {total_cleared} records from {map_name}',
            'map_name': map_name,
            'cleared_counts': cleared_counts,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error clearing nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to clear nodes: {str(e)}'
        }), 500

@app.route('/list-maps', methods=['GET'])
def list_maps():
    """List all available maps/databases"""
    try:
        if not os.path.exists(DATABASE_DIR):
            os.makedirs(DATABASE_DIR)
            return jsonify({
                'status': 'success',
                'maps': [],
                'total': 0,
                'message': 'No databases created yet'
            })
        
        # Get all .db files in the directory
        db_files = [f for f in os.listdir(DATABASE_DIR) if f.endswith('.db')]
        
        maps_info = []
        for db_file in db_files:
            map_name = os.path.splitext(db_file)[0].replace('_', ' ')
            db_path = os.path.join(DATABASE_DIR, db_file)
            db_size = os.path.getsize(db_path) / (1024 * 1024)  # MB
            
            # Get record counts
            try:
                conn = sqlite3.connect(db_path)
                cursor = conn.cursor()
                
                # Count total records
                tables = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
                total_records = 0
                for table in tables:
                    cursor.execute(f"SELECT COUNT(*) FROM {table}")
                    total_records += cursor.fetchone()[0]
                
                conn.close()
                
                maps_info.append({
                    'map_name': map_name,
                    'database_file': db_file,
                    'file_size_mb': round(db_size, 2),
                    'total_records': total_records,
                    'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
                })
            except:
                maps_info.append({
                    'map_name': map_name,
                    'database_file': db_file,
                    'file_size_mb': round(db_size, 2),
                    'total_records': 'Unknown',
                    'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
                })
        
        return jsonify({
            'status': 'success',
            'maps': maps_info,
            'total': len(maps_info),
            'database_directory': DATABASE_DIR,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error listing maps: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to list maps: {str(e)}'
        }), 500

@app.route('/delete-map', methods=['DELETE'])
def delete_map():
    """Delete a map database file"""
    try:
        map_name = request.args.get('map_name', 'default')
        
        if not map_name:
            return jsonify({
                'status': 'error',
                'message': 'Map name is required'
            }), 400
        
        db_path = get_database_path(map_name)
        
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}'
            }), 404
        
        # Delete the database file
        os.remove(db_path)
        logger.info(f"🗑️ Deleted database for map '{map_name}': {db_path}")
        
        return jsonify({
            'status': 'success',
            'message': f'Successfully deleted database for map: {map_name}',
            'deleted_file': os.path.basename(db_path),
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error deleting map: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to delete map: {str(e)}'
        }), 500

@app.errorhandler(404)
def not_found(error):
    return jsonify({
        'status': 'error',
        'message': 'Endpoint not found'
    }), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({
        'status': 'error',
        'message': 'Internal server error'
    }), 500

if __name__ == '__main__':
    print("🚀 Starting AMR Map Editor API Server...")
    print("📁 Database system: Per-map databases")
    print(f"📂 Database directory: {os.path.abspath(DATABASE_DIR)}")
    print("✅ API ready!")
    print("\n🌐 Starting Flask server...")
    print("   📍 Local: http://localhost:5000")
    print("   🌍 Network: http://0.0.0.0:5000")
    
    # Create database directory if it doesn't exist
    if not os.path.exists(DATABASE_DIR):
        os.makedirs(DATABASE_DIR)
        print(f"📁 Created database directory: {DATABASE_DIR}")
    
    # Run on all interfaces (0.0.0.0) instead of just localhost
    app.run(debug=True, host='0.0.0.0', port=5000)