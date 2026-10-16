# Scenario files in this world

Arena scenarios in this tutorial are JSON files. Each scenario has its own
directory and the file inside it must be named `scenario.json`:

```text
scenarios/
├── default/
│   └── scenario.json          complete working example
└── student_skeleton/
    └── scenario.json          small editable starting point
```

Read `../SCENARIO_GUIDE.md` before editing. JSON cannot contain comments, so
the explanations live in that Markdown file instead of inside `scenario.json`.

Select a scenario using its directory name:

```bash
scenario_file:=default
scenario_file:=student_skeleton
```
