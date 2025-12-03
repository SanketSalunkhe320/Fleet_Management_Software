## Fleet Manager Dashboard — Copilot instructions

Short, actionable guidance to help AI coding agents be productive in this repo.

- Big picture
  - Frontend: React + Vite (entry: `src/App.jsx`, pages in `src/pages/`, components in `src/components/`). Dev server: `npm run dev` (Vite, usually at http://localhost:5173).
  - Backend: Single Python process that runs a Flask REST API and two WebSocket servers. Main entry: `backend/app.py`. Database layer: `backend/database.py` (SQLite `amr_tasks.db`).
  - Integrations: Frontend connects to ROS via websocket at ws://localhost:9090 (in `src/pages/RealTime.jsx`). Frontend also connects to the task manager client WS on ws://localhost:8766. Robots connect to the robot WS on ws://localhost:8765.

- How to run / dev commands (verified from repo files)
  - Frontend (in repo root):
    - npm install
    - npm run dev  # starts Vite dev server
  - Backend (in `backend/`):
    - python3 backend/app.py  # starts Flask on :5000 and WS servers on ports 8765 (robots) / 8766 (clients)
  - Notes: the backend creates `uploads/tasks` and uses `amr_tasks.db` in repo root by default.

- Important runtime/ports to know
  - Vite frontend: 5173 (dev)
  - Flask REST API: 5000 (backend/app.py)
  - ROS websocket bridge: 9090 (RealTime.jsx subscribes to `/map`)
  - AMR WebSocket servers: 8765 (robot connections) and 8766 (client connections)

- Project-specific patterns & conventions
  - WebSocket message types (see `backend/app.py` and `src/pages/RealTime.jsx`):
    - Client -> server examples: `{ type: 'get_tasks' }`, `{ type: 'get_robots' }`, `{ type: 'new_task', taskData: {...} }`, `{ type: 'assign_task', taskId, robotId }`, `{ type: 'disconnect_robot', robotId }`.
    - Server -> client examples: `{ type: 'tasks_list', tasks: [...] }`, `{ type: 'robots_list', robots: [...] }`, `{ type: 'task_update', task: {...} }`, `{ type: 'robot_status', robotId, status }`.
  - AMR IDs: code uses IDs like `AMR-1`; UI assumes certain robot IDs when auto-disconnecting (see `app.py` auto-disconnect for `'AMR-1'`).
  - Database: `backend/database.py` stores tasks and robots as JSON strings for nodes/arrows/zones and uses fields `z_axis` and `z_position` (Z support is available).
  - Authentication: a lightweight JWT is implemented in `backend/app.py` with `app.config['SECRET_KEY']` — tests and secure secrets are not configured; treat as development-only.

- Files to inspect when making changes
  - `backend/app.py` — main orchestration and WS message handling (heavy logic). Update here for task/robot behavior.
  - `backend/database.py` — SQLite schema and helpers (tasks, robots, history). Use these helpers rather than direct DB writes.
  - `src/pages/RealTime.jsx` — ROS subscriptions, canvas/map code, and the client WS implementation. Many UI behaviors and message handling live here (map coordinates, zoom/pan, node/arrow/zone UX).
  - `src/App.jsx` and `src/components/ProtectedRoute.jsx` — routing and auth flow used across the app.

- Small examples to follow
  - Creating a new task (frontend -> backend client WS):
    { type: 'new_task', taskData: { mapName, amrName, nodes, arrows, zones, zAxis } }
  - Listening for updates (client WS handler in `RealTime.jsx`): handle `tasks_list`, `robots_list`, `task_update`, `robot_status` and update React state accordingly.

- What NOT to change without extra care
  - WebSocket message shapes: the frontend and backend tightly couple to the message fields (mapName vs map_name, zAxis vs z_axis). If you change a field, update both sides and the DB serialization.
  - Database schema: `backend/database.py` includes indexes and foreign keys; altering schema requires migration or data handling.

If anything here is unclear or you'd like the instructions to contain more examples (e.g., sample WS payloads for every code path or explicit test commands), tell me which area to expand and I will iterate.
