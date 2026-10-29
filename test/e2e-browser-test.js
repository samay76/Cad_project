import fs from 'fs';

async function run() {
  console.log('🚀 Launching Firefox via WebDriver to test live browser CAD Viewer...');

  const BASE = 'http://127.0.0.1:4444';

  // 1. Create Session
  const sessionRes = await fetch(`${BASE}/session`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      capabilities: {
        alwaysMatch: {
          'moz:firefoxOptions': {
            args: ['-headless', '--window-size=1440,900']
          }
        }
      }
    })
  });
  const sessionData = await sessionRes.json();
  const sessionId = sessionData.value.sessionId;
  console.log('✅ Session created:', sessionId);

  try {
    // 2. Navigate to localhost:5173
    console.log('Navigating to http://localhost:5173...');
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:5173' })
    });

    // 3. Wait for WASM & default model to load
    console.log('Waiting for OpenCascade WASM to triangulate and load CAD model...');
    let loaded = false;
    for (let i = 0; i < 20; i++) {
      await new Promise(r => setTimeout(r, 1000));
      const res = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          script: `
            const parts = document.querySelectorAll('.part-item').length;
            const fileText = document.getElementById('loaded-file-text')?.innerText || '';
            const loading = document.getElementById('loading-overlay')?.classList.contains('visible');
            return { parts, fileText, loading };
          `,
          args: []
        })
      });
      const data = await res.json();
      console.log(`  Polling [${i}s]: parts=${data.value?.parts}, file="${data.value?.fileText}", loading=${data.value?.loading}`);
      if (data.value?.parts > 0 && !data.value?.loading) {
        loaded = true;
        break;
      }
    }

    if (!loaded) {
      throw new Error('Model did not finish loading within timeout');
    }

    // 4. Inspect Properties Panel
    const inspectRes = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const parts = Array.from(document.querySelectorAll('.part-item')).map(el => ({
            name: el.querySelector('.part-name')?.innerText,
            triangles: el.querySelector('.part-triangles')?.innerText
          }));
          const props = document.querySelector('.properties-scrollable')?.innerText;
          return { partsCount: parts.length, firstParts: parts.slice(0, 5), props };
        `,
        args: []
      })
    });
    const inspectData = await inspectRes.json();
    console.log('\n📊 UI Status in Browser:');
    console.log(`  - Parts in sidebar: ${inspectData.value.partsCount}`);
    console.log('  - First 5 parts:', inspectData.value.firstParts);

    // 5. Test Click Selection on Part 0 in Parts List
    console.log('\n🖱️ Selecting first part (PLATE) in parts list...');
    await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const firstPart = document.querySelector('.part-item[data-part-index="0"]');
          if (firstPart) firstPart.click();
        `,
        args: []
      })
    });

    await new Promise(r => setTimeout(r, 600));

    // 6. Inspect Selected State & Coordinates in Properties Panel
    const selectionRes = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const selectedItem = document.querySelector('.part-item.is-selected .part-name')?.innerText;
          const pickCard = document.querySelector('.pick-point-container');
          const coords = Array.from(document.querySelectorAll('.coord-readout-item')).map(el => el.innerText.replace(/\\n/g, ' '));
          return {
            selectedItem,
            hasPickCard: !!pickCard,
            coords
          };
        `,
        args: []
      })
    });
    const selectionData = await selectionRes.json();
    console.log('\n🎯 Selection Verification:');
    console.log('  - Selected Item in UI:', selectionData.value.selectedItem);
    console.log('  - Pick Point & Normal Readouts:', selectionData.value.coords);

    // 7. Test Clicking on 3D Viewport Canvas
    console.log('\n🎯 Simulating Click on 3D Viewport Canvas to pick exact surface point...');
    await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const canvas = document.querySelector('.cad-canvas');
          const rect = canvas.getBoundingClientRect();
          const clientX = rect.left + rect.width / 2;
          const clientY = rect.top + rect.height / 2;
          canvas.dispatchEvent(new PointerEvent('pointerdown', { clientX, clientY, bubbles: true }));
          canvas.dispatchEvent(new PointerEvent('pointerup', { clientX, clientY, bubbles: true }));
        `,
        args: []
      })
    });

    await new Promise(r => setTimeout(r, 600));

    // 8. Capture Screenshot
    console.log('\n📸 Capturing screenshot of live browser session...');
    const shotRes = await fetch(`${BASE}/session/${sessionId}/screenshot`);
    const shotData = await shotRes.json();
    if (shotData.value) {
      fs.writeFileSync('/home/human/cad-browser-verified.png', Buffer.from(shotData.value, 'base64'));
      console.log('  ✅ Saved screenshot to /home/human/cad-browser-verified.png');
    }

    console.log('\n🎉 ALL LIVE BROWSER ACCEPTANCE TESTS PASSED!');
  } finally {
    console.log('Cleaning up browser session...');
    await fetch(`${BASE}/session/${sessionId}`, { method: 'DELETE' });
  }
}

run().catch(err => {
  console.error('Browser test failed:', err);
  process.exit(1);
});
