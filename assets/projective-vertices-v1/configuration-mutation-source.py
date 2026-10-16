#!/workspace/.rustdrive-tools/demo-venv/bin/python
"""Deliberately wrong transport for checking rejection-test sensitivity."""
import json,subprocess,sys

binary='/workspace/rust_drive/integrations/projective-depth/target/release/rustdriving-projective-depth'
child=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=sys.stdout,stderr=sys.stderr,text=True)
for line in sys.stdin:
    try:
        request=json.loads(line)
        if request.get('kind')=='Configure' and request.get('geometry')=='unknown':
            del request['geometry']
            line=json.dumps(request)+'\n'
    except (ValueError,AttributeError):
        pass
    child.stdin.write(line)
    child.stdin.flush()
child.stdin.close()
sys.exit(child.wait())
