import fs from 'fs';

async function run() {
  console.log('🧪 Testing STEP file input loading feature...');
  const BASE = 'http://127.0.0.1:4444';

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

  try {
    // 1. Navigate to localhost:3000
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:3000' })
    });

    await new Promise(r => setTimeout(r, 2000));

    // 2. Test typing a STEP file path into the new input box and clicking Load!
    console.log('Typing "/samples/cube.stp" into #cad-url-input...');
    await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const input = document.getElementById('cad-url-input');
          const btn = document.getElementById('btn-load-url');
          input.value = '/samples/cube.stp';
          btn.click();
        `,
        args: []
      })
    });

    // Wait for cube.stp to load
    let cubeLoaded = false;
    for (let i = 0; i < 15; i++) {
      await new Promise(r => setTimeout(r, 500));
      const res = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          script: `
            const fileText = document.getElementById('loaded-file-text')?.innerText || '';
            const parts = document.querySelectorAll('.part-item').length;
            const loading = document.getElementById('loading-overlay')?.classList.contains('visible');
            return { fileText, parts, loading };
          `,
          args: []
        })
      });
      const data = await res.json();
      console.log(`  Polling: file="${data.value?.fileText}", parts=${data.value?.parts}, loading=${data.value?.loading}`);
      if (data.value?.fileText === 'cube.stp' && data.value?.parts === 1 && !data.value?.loading) {
        cubeLoaded = true;
        break;
      }
    }

    if (!cubeLoaded) {
      throw new Error('Failed to load cube.stp via input box');
    }
    console.log('✅ Successfully loaded STEP file via #cad-url-input!');

    // 3. Test URL Query Parameter (?step=/samples/dm1-oc-214.stp)
    console.log('\nTesting URL query param "?step=/samples/dm1-oc-214.stp"...');
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:3000/?step=/samples/dm1-oc-214.stp' })
    });

    let dm1Loaded = false;
    for (let i = 0; i < 20; i++) {
      await new Promise(r => setTimeout(r, 1000));
      const res = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          script: `
            const fileText = document.getElementById('loaded-file-text')?.innerText || '';
            const parts = document.querySelectorAll('.part-item').length;
            const loading = document.getElementById('loading-overlay')?.classList.contains('visible');
            return { fileText, parts, loading };
          `,
          args: []
        })
      });
      const data = await res.json();
      console.log(`  Polling query param: file="${data.value?.fileText}", parts=${data.value?.parts}, loading=${data.value?.loading}`);
      if (data.value?.fileText === 'dm1-oc-214.stp' && data.value?.parts > 0 && !data.value?.loading) {
        dm1Loaded = true;
        break;
      }
    }

    if (!dm1Loaded) {
      throw new Error('Failed to load STEP file via query parameter');
    }
    console.log('✅ Successfully loaded STEP file via ?step= query parameter!');

    // Capture screenshot
    const shotRes = await fetch(`${BASE}/session/${sessionId}/screenshot`);
    const shotData = await shotRes.json();
    if (shotData.value) {
      fs.writeFileSync('/home/human/cad-step-input-verified.png', Buffer.from(shotData.value, 'base64'));
      console.log('📸 Saved verification screenshot to /home/human/cad-step-input-verified.png');
    }

    console.log('\n🎉 ALL STEP INPUT TESTS PASSED PERFECTLY!');
  } finally {
    await fetch(`${BASE}/session/${sessionId}`, { method: 'DELETE' });
  }
}

run().catch(err => {
  console.error('Test failed:', err);
  process.exit(1);
});
