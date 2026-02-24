from openai import AzureOpenAI
import os
import base64
import sys
from dotenv import load_dotenv

# ---------------------------------------------------------------------------
# Cached client -- reused across calls (saves ~200-400ms connection setup)
# ---------------------------------------------------------------------------
_client = None

def _get_client():
    """Get or create cached Azure OpenAI client."""
    global _client
    if _client is None:
        _client = AzureOpenAI(
            api_key=os.environ['OPENAI_API_KEY'],
            api_version=os.environ['API_VERSION'],
            azure_endpoint=os.environ['OPENAI_API_BASE'],
            organization=os.environ['OPENAI_ORGANIZATION']
        )
    return _client

SYSTEM_PROMPT = (
    "Role: Small ground robot vision system. "
    "Rule 1: You cannot drive over ANY objects. "
    "Rule 2: Scan the floor immediately ahead for obstacles. "
    "Rule 3: Find the widest clear path."
    "Rule 4: Avoid bad lighting conditions."
    "Output: EXACTLY ONE UPPERCASE LETTER (F, B, L, R). No text, no markdown."
)

USER_PROMPT = ( "Where is the biggest gap or open path? Respond with one letter: F, B, L, R. Don't get stuck in a loop if front is blocked or the path is unclear or not well lit, look around in other directions. Ajust direction if you see a better gap." 
               "Always pick the clearest path, even if it means reversing or going sideways."
               )

def get_driving_command(dynamic_prompt, image_path=None):
    """Send an image to the LLM and return a single driving command letter (F/B/L/R/S).

    Args:
        dynamic_prompt: The text prompt including history/memory.
        image_path: Optional path to an image file. Defaults to images/image.jpg.
    """
    if image_path is None:
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        image_path = os.path.join(project_root, 'images', 'image.jpg')

    # Safety Net 1: Check if image actually exists before reading
    if not os.path.exists(image_path):
        print(f"[llm] Error: Image not found at {image_path}")
        return 'S'

    with open(image_path, 'rb') as f:
        imagedata = base64.b64encode(f.read()).decode('ascii')

    client = _get_client()

    messages = [
        {
            "role": "system",
            "content": SYSTEM_PROMPT
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "text", 
                    "text": dynamic_prompt
                },
                {
                    "type": "image_url", 
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{imagedata}",
                        "detail": "low" 
                    }
                }
            ]
        }
    ]

    try:
        response = client.chat.completions.create(
            model=os.environ['MODEL'],
            messages=messages,
            # max_tokens=5,    
            # temperature=0.0   
        )

        raw = response.choices[0].message.content.strip().upper()
        
        # Safety Net 2: Aggressive output filtering
        valid_commands = ['F', 'B', 'L', 'R', 'S']
        for char in raw:
            if char in valid_commands:
                return char
                
        print(f"[llm] Unrecognized output: '{raw}'")
        return 'S'

    except Exception as e:
        # Safety Net 3: Prevent crash on Wi-Fi drop or Azure timeout
        print(f"[llm] Azure API Error: {e}")
        return 'S'

# ---------------------------------------------------------------------------
# Standalone Testing Block
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    load_dotenv(os.path.join(project_root, '.env'))

    if len(sys.argv) < 2:
        print("Usage: python modules/openai_vision.py <image_path>")
        print("  e.g. python modules/openai_vision.py test_images/hallway.jpg")
        sys.exit(1)

    img = sys.argv[1]
    if not os.path.isfile(img):
        print(f"File not found: {img}")
        sys.exit(1)

    # Uses the standalone USER_PROMPT for simple testing
    cmd = get_driving_command(dynamic_prompt=USER_PROMPT, image_path=img)
    print(f"LLM says: {cmd}")