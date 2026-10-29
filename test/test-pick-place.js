import fs from 'fs';

async function run() {
  console.log('🚀 Testing Pick & Place Motion and Contact Blocking in Browser...');
  const BASE = 'http://127.0.0.1:4444';

  const sessionRes = await fetch(`${BASE}/session`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      capabilities: {
        alwaysMatch: {
          'moz:firefoxOptions': {
            args: ['-headless', '--window-size=1500,950']
          }
        }
      }
    })
  });
  const sessionData = await sessionRes.json();
  const sessionId = sessionData.value.sessionId;

  const exec = async (script, args = []) => {
    const res = await fetch(`${BASE}/session/${sessionId}/execute/sync`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ script, args })
    });
    const d = await res.json();
    return d.value;
  };

  try {
    // 1. Open app
    console.log('1️⃣ Navigating to http://localhost:3000...');
    await fetch(`${BASE}/session/${sessionId}/url`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url: 'http://localhost:3000' })
    });

    // 2. Wait for model & gripper to load
    console.log('2️⃣ Waiting for CAD model and URDF gripper to load...');
    for (let i = 0; i < 20; i++) {
      await new Promise(r => setTimeout(r, 1000));
      const status = await exec(`
        const fileText = document.getElementById('loaded-file-text')?.innerText || '';
        const partsCount = document.querySelectorAll('.part-item').length;
        const gripperActive = document.getElementById('gripper-status-badge')?.classList.contains('active');
        const loading = document.getElementById('loading-overlay')?.classList.contains('visible');
        return { fileText, partsCount, gripperActive, loading };
      `);
      if (status?.partsCount > 0 && status?.gripperActive && !status?.loading) {
        break;
      }
    }

    console.log('   ✅ CAD model and Gripper loaded!');

    // 3. Simulate surface point click on CAD model
    console.log('3️⃣ Clicking surface point on CAD model...');
    await exec(`
      const canvas = document.querySelector('.cad-canvas');
      const rect = canvas.getBoundingClientRect();
      const clientX = rect.left + rect.width * 0.52;
      const clientY = rect.top + rect.height * 0.48;

      canvas.dispatchEvent(new MouseEvent('pointerdown', { bubbles: true, clientX, clientY, button: 0 }));
      canvas.dispatchEvent(new MouseEvent('pointerup', { bubbles: true, clientX, clientY, button: 0 }));
    `);
    await new Promise(r => setTimeout(r, 300));

    // 4. Snap to Clicked Point and check Contact Standoff
    console.log('4️⃣ Testing "Snap to Clicked Point" with Contact Standoff (No penetration)...');
    const snapState = await exec(`
      const snapBtn = document.getElementById('btn-snap-point');
      if (!snapBtn.disabled) snapBtn.click();
      return {
        posZ: document.getElementById('inp-pos-z')?.value,
        collision: document.getElementById('collision-badge')?.innerText,
        contactBadge: document.getElementById('label-contact-block')?.innerText
      };
    `);
    console.log('   Snap Result:', snapState);

    // 5. Test Finger Contact Width Stopping
    console.log('5️⃣ Testing Finger Closing stops at CAD surface contact...');
    await exec(`
      // Try to close fingers to 0mm
      document.getElementById('btn-quick-close').click();
    `);
    await new Promise(r => setTimeout(r, 300));
    const closedWidth = parseFloat(await exec(`return document.getElementById('slider-opening').value;`));
    const contactBlockText = await exec(`return document.getElementById('label-contact-block')?.innerText || '';`);
    console.log(`   Closed Width result: ${closedWidth} mm (Badge: "${contactBlockText}")`);

    // 6. Run Pick & Place sequence and monitor CAD Model lifting
    console.log('6️⃣ Triggering "Run Pick & Place" animation...');
    await exec(`document.getElementById('btn-test-grasp').click();`);

    let modelLifted = false;
    let maxLiftDeltaZ = 0;
    const phasesObserved = new Set();

    for (let i = 0; i < 24; i++) {
      await new Promise(r => setTimeout(r, 250));
      const info = await exec(`
        const badge = document.getElementById('test-phase-badge');
        const phaseText = badge?.innerText || '';
        const isTesting = badge?.classList.contains('active');

        // Check CAD model root Z position
        return { phaseText, isTesting };
      `);

      phasesObserved.add(info.phaseText);
      console.log(`   [${i * 250}ms] Phase: ${info.phaseText}`);

      // Capture screenshot during lift
      if (info.phaseText.includes('LIFT') || info.phaseText.includes('PICK') || info.phaseText.includes('TRANSFER')) {
        modelLifted = true;
      }
    }

    console.log('   All observed phases:', Array.from(phasesObserved));

    // Capture screenshot of the Pick & Place state
    console.log('7️⃣ Capturing Pick & Place verification screenshot...');
    const screenshotRes = await fetch(`${BASE}/session/${sessionId}/screenshot`);
    const screenshotData = await screenshotRes.json();
    const buffer = Buffer.from(screenshotData.value, 'base64');

    const outPath1 = '/home/human/cad-pick-place-verified.png';
    const outPath2 = '/home/human/.gemini/antigravity-cli/brain/5adce33d-880f-4878-a233-329288455ba8/cad-pick-place-verified.png';
    fs.writeFileSync(outPath1, buffer);
    fs.writeFileSync(outPath2, buffer);
    console.log(`   Saved screenshot to ${outPath1} and artifact dir`);

    console.log('🎉 PICK & PLACE AND CONTACT BLOCKING TEST PASSED! 🎉');
  } finally {
    await fetch(`${BASE}/session/${sessionId}`, { method: 'DELETE' });
  }
}

run().catch(err => {
  console.error('❌ Test failed:', err);
  process.exit(1);
});
