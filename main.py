from flask import Flask, render_template, redirect, url_for
import os
import markdown2
from pathlib import Path

app = Flask(__name__)
app.secret_key = os.environ.get("SESSION_SECRET", "ros2_m1_tutorial_secret")

@app.route('/')
def index():
    return redirect(url_for('tutorial_index'))

@app.route('/tutorial/')
def tutorial_index():
    # List all tutorial files
    tutorial_dir = Path('./tutorial')
    tutorial_files = []
    
    for file in sorted(tutorial_dir.glob('*.md')):
        # Extract number and title from filename
        name = file.stem
        if name[0].isdigit() and name[1].isdigit() and name[2] == '_':
            number = int(name[:2])
            title = name[3:].replace('_', ' ').title()
            tutorial_files.append({
                'number': number,
                'title': title,
                'path': file.name
            })
    
    return render_template('index.html', tutorials=tutorial_files)

@app.route('/tutorial/<filename>')
def tutorial_page(filename):
    try:
        # Read the markdown file
        file_path = Path(f'./tutorial/{filename}')
        if not file_path.exists():
            return render_template('error.html', message=f"Tutorial file '{filename}' not found"), 404
            
        with open(file_path, 'r') as file:
            content = file.read()
            
        # Convert Markdown to HTML
        html_content = markdown2.markdown(
            content, 
            extras=["fenced-code-blocks", "tables", "header-ids"]
        )
        
        # Replace bash prompt symbols with >>
        import re
        # Replace all $ prompts with >>
        pattern = r'<code class="language-bash">(.*?)(\$\s+)'
        replacement = r'<code class="language-bash">\1>> '
        html_content = re.sub(pattern, replacement, html_content, flags=re.DOTALL)
        
        # Also look for ```bash blocks that don't have explicit $ prompts
        # but might represent shell commands
        def process_bash_code(match):
            code = match.group(1)
            # If there are multiple lines and no prompt
            if '\n' in code and not re.search(r'>>\s+', code):
                # Add >> prompt to each line that looks like a command
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    # Skip empty lines, comments, and continued lines
                    if line.strip() and not line.strip().startswith('#') and not line.strip().startswith('|'):
                        # Make sure it's not already processed
                        if not line.strip().startswith('>>'):
                            lines[i] = '>> ' + line
                code = '\n'.join(lines)
            return f'<code class="language-bash">{code}'
            
        pattern = r'<code class="language-bash">(.*?)'
        html_content = re.sub(pattern, process_bash_code, html_content, flags=re.DOTALL)
        
        # Get previous and next tutorial links
        current_num = int(filename[:2])
        prev_file = None
        next_file = None
        
        tutorial_dir = Path('./tutorial')
        for file in sorted(tutorial_dir.glob('*.md')):
            name = file.stem
            if name[0].isdigit() and name[1].isdigit() and name[2] == '_':
                num = int(name[:2])
                if num == current_num - 1:
                    prev_file = {'path': file.name, 'title': name[3:].replace('_', ' ').title()}
                elif num == current_num + 1:
                    next_file = {'path': file.name, 'title': name[3:].replace('_', ' ').title()}
        
        return render_template(
            'tutorial.html',
            content=html_content,
            title=filename[3:-3].replace('_', ' ').title(),
            prev=prev_file,
            next=next_file
        )
        
    except Exception as e:
        return render_template('error.html', message=str(e)), 500

@app.route('/code/<path:filepath>')
def code_view(filepath):
    try:
        path = Path(f"./ros2_m1_sim/{filepath}")
        if not path.exists() or not path.is_file():
            return render_template('error.html', message=f"File '{filepath}' not found"), 404
            
        with open(path, 'r') as file:
            content = file.read()
            
        # Determine file type for syntax highlighting
        file_extension = path.suffix.lstrip('.')
        language = {
            'py': 'python',
            'launch': 'python',
            'xml': 'xml',
            'yaml': 'yaml',
            'md': 'markdown'
        }.get(file_extension, 'plaintext')
        
        return render_template(
            'code.html',
            content=content,
            filepath=filepath,
            language=language
        )
            
    except Exception as e:
        return render_template('error.html', message=str(e)), 500

@app.route('/code/')
def code_index():
    structure = build_directory_structure('./ros2_m1_sim')
    return render_template('code_index.html', structure=structure)

def build_directory_structure(base_path):
    """Build a nested dictionary representing the directory structure."""
    path = Path(base_path)
    result = {'name': path.name, 'path': str(path), 'type': 'dir', 'children': []}
    
    try:
        for item in sorted(path.iterdir()):
            if item.name.startswith('.'):
                continue
                
            if item.is_dir():
                result['children'].append(build_directory_structure(item))
            else:
                result['children'].append({
                    'name': item.name, 
                    'path': str(item), 
                    'type': 'file'
                })
    except Exception:
        pass
        
    return result

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)