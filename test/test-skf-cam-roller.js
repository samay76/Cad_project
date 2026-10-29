import fs from 'fs';

async function run() {
  console.log('🧪 Starting live test for SKF Cam Roller STEP file (/home/human/nferent_cad/skf_cam_roller_305703_c-2z_0.stp)...');
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
    // 1. Open app
    console.log('Navigating to http://localhost:3000...');
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:3000' })
    });

    // 2. Wait for SKF Cam Roller to load automatically
    console.log('Waiting for SKF Cam Roller to load & triangulate...');
    let initialLoaded = false;
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
            const errorToast = document.getElementById('error-toast')?.classList.contains('visible');
            const errorMsg = document.getElementById('error-message')?.innerText || '';
            return { fileText, parts, loading, errorToast, errorMsg };
          `,
          args: []
        })
      });
      const data = await res.json();
      console.log(`  Polling [${i}s]: file="${data.value?.fileText}", parts=${data.value?.parts}, loading=${data.value?.loading}, error=${data.value?.errorToast}`);
      if (data.value?.errorToast) {
        throw new Error(`CAD Loading Error encountered: ${data.value.errorMsg}`);
      }
      if (data.value?.parts === 6 && !data.value?.loading) {
        initialLoaded = true;
        break;
      }
    }

    if (!initialLoaded) {
      throw new Error('Initial load timed out or parts != 6');
    }
    console.log('✅ Initial load of SKF Cam Roller passed with 6 parts!');

    // 3. Test explicit input with full absolute path /home/human/nferent_cad/skf_cam_roller_305703_c-2z_0.stp
    console.log('\nTesting direct path input: "/home/human/nferent_cad/skf_cam_roller_305703_c-2z_0.stp"...');
    await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const input = document.getElementById('cad-url-input');
          const btn = document.getElementById('btn-load-url');
          input.value = '/home/human/nferent_cad/skf_cam_roller_305703_c-2z_0.stp';
          btn.click();
        `,
        args: []
      })
    });

    let reloadSuccess = false;
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
            const errorToast = document.getElementById('error-toast')?.classList.contains('visible');
            const errorMsg = document.getElementById('error-message')?.innerText || '';
            return { fileText, parts, loading, errorToast, errorMsg };
          `,
          args: []
        })
      });
      const data = await res.json();
      if (data.value?.errorToast) {
        throw new Error(`CAD Loading Error on direct path input: ${data.value.errorMsg}`);
      }
      if (data.value?.parts === 6 && !data.value?.loading) {
        reloadSuccess = true;
        break;
      }
    }

    if (!reloadSuccess) {
      throw new Error('Direct path load failed');
    }
    console.log('✅ Successfully loaded via absolute path input with 0 errors!');

    // 4. Test selecting part #0 and checking coordinates
    console.log('\nSelecting Part #0...');
    await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const part0 = document.querySelector('.part-item[data-part-index="0"]');
          if (part0) part0.click();
        `,
        args: []
      })
    });

    await new Promise(r => setTimeout(r, 600));

    const checkRes = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        script: `
          const selectedName = document.querySelector('.part-item.is-selected .part-name')?.innerText;
          const readouts = Array.from(document.querySelectorAll('.coord-readout-item')).map(el => el.innerText.replace(/\\n/g, ' '));
          const bbox = Array.from(document.querySelectorAll('.coord-val')).map(el => el.innerText);
          return { selectedName, readouts, bbox };
        `,
        args: []
      })
    });
    const checkData = await checkRes.json();
    console.log('Selected Part:', checkData.value?.selectedName);
    console.log('Bounding Box in mm:', checkData.value?.bbox);
    console.log('Pick Point Coordinates:', checkData.value?.readouts);

    // 5. Capture screenshot
    const shotRes = await fetch(`${BASE}/session/${sessionId}/screenshot`);
    const shotData = await shotRes.json();
    if (shotData.value) {
      fs.writeFileSync('/home/human/skf-cam-roller-verified.png', Buffer.from(shotData.value, 'base64'));
      console.log('📸 Saved screenshot to /home/human/skf-cam-roller-verified.png');
    }

    console.log('\n🎉 TEST SUCCEEDED WITH ZERO ERRORS!');
  } finally {
    await fetch(`${BASE}/session/${sessionId}`, { method: 'DELETE' });
  }
}

run().catch(err => {
  console.error('Test failed:', err);
  process.exit(1);
});
